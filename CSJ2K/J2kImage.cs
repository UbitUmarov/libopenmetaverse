// Copyright (c) 2007-2016 CSJ2K contributors.
// Licensed under the BSD 3-Clause License.

using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;

using CSJ2K.Color;
using CSJ2K.Icc;
using CSJ2K.j2k.codestream;
using CSJ2K.j2k.codestream.reader;
using CSJ2K.j2k.codestream.writer;
using CSJ2K.j2k.decoder;
using CSJ2K.j2k.encoder;
using CSJ2K.j2k.entropy.decoder;
using CSJ2K.j2k.entropy.encoder;
using CSJ2K.j2k.fileformat;
using CSJ2K.j2k.fileformat.reader;
using CSJ2K.j2k.fileformat.writer;
using CSJ2K.j2k.image;
using CSJ2K.j2k.image.forwcomptransf;
using CSJ2K.j2k.image.input;
using CSJ2K.j2k.image.invcomptransf;
using CSJ2K.j2k.io;
using CSJ2K.j2k.quantization.dequantizer;
using CSJ2K.j2k.quantization.quantizer;
using CSJ2K.j2k.roi;
using CSJ2K.j2k.roi.encoder;
using CSJ2K.j2k.util;
using CSJ2K.j2k.wavelet.analysis;
using CSJ2K.j2k.wavelet.synthesis;
using CSJ2K.Util;


namespace CSJ2K
{
    public class J2kImage
    {

        #region Static Decoder Methods

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Image FromBytes(byte[] j2kdata, ParameterList parameters = null, bool stripAlpha = false, int Discard = -1)
        {
            using MemoryStream stream = new(j2kdata);
            return FromStream(stream, parameters, stripAlpha, Discard);
        }

        public static Image FromStream(Stream stream, ParameterList parameters = null, bool stripAlpha = false, int Discard = -1)
        {
            RandomAccessIO in_stream = new ISRandomAccessIO(stream);

            var pl = parameters ?? new ParameterList(GetDefaultDecoderParameterList(decoder_pinfo));

            // **** File Format ****
            // If the codestream is wrapped in the jp2 fileformat, Read the
            // file format wrapper
            FileFormatReader ff = new FileFormatReader(in_stream);
            ff.readFileFormat();
            if (ff.JP2FFUsed)
                in_stream.seek(ff.FirstCodeStreamPos);

            // +----------------------------+
            // | Instantiate decoding chain |
            // +----------------------------+

            // **** Header decoder ****
            // Instantiate header decoder and read main header 
            HeaderInfo hi = new HeaderInfo();
            HeaderDecoder hd;
            try
            {
                hd = new HeaderDecoder(in_stream, pl, hi);
            }
            catch (EndOfStreamException e)
            {
                throw new InvalidOperationException("Codestream too short or bad header, unable to decode.", e);
            }

            int nCompCod = hd.NumComps;
            int nTiles = hi.sizValue.NumTiles;
            DecoderSpecs decSpec = hd.DecoderSpecs;

            // Get demixed bitdepths
            int[] depth = new int[nCompCod];
            for (int i = 0; i < nCompCod; i++)
            {
                depth[i] = hd.getOriginalBitDepth(i);
            }

            // **** Discard (reduced resolution) ****
            // the target resolution must be known to the bitstream reader so the subband tree, and with it
            // the tile sizes, are built for it; the caller's "res" is restored afterwards
            string prevRes = null;
            bool setRes = false;
            if (Discard >= 0)
            {
                int targetRes = decSpec.dls.Min;
                if (Discard == 0)
                    targetRes = 0;
                else
                {
                    int bw = MathUtil.log2(hd.ImgWidth);
                    int bh = MathUtil.log2(hd.ImgHeight);
                    int d = (bw + bh - Discard) / 2;
                    if (d > 0)
                    {
                        if (d > bh - 1)
                            d = bh - 1;
                        if (d > bw - 1)
                            d = bw - 1;

                        if (d > 0)
                        {
                            targetRes -= d;
                            if (targetRes < 0)
                                targetRes = 0;
                        }
                    }
                }
                pl.TryGetValue("res", out prevRes);
                pl["res"] = targetRes.ToString();
                setRes = true;
            }

            // **** Bit stream reader ****
            BitstreamReaderAgent breader;
            try
            {
                breader = BitstreamReaderAgent.createInstance(in_stream, hd, pl, decSpec, false, hi);
            }
            catch (IOException e)
            {
                throw new InvalidOperationException("Error while reading bit stream header or parsing packets.", e);
            }
            catch (ArgumentException e)
            {
                throw new InvalidOperationException("Cannot instantiate bit stream reader.", e);
            }
            finally
            {
                if (setRes)
                {
                    if (prevRes == null)
                        pl.Remove("res");
                    else
                        pl["res"] = prevRes;
                }
            }

            // **** Entropy decoder ****
            EntropyDecoder entdec;
            try
            {
                entdec = hd.createEntropyDecoder(breader, pl);
            }
            catch (ArgumentException e)
            {
                throw new InvalidOperationException("Cannot instantiate entropy decoder.", e);
            }

            // **** ROI de-scaler ****
            ROIDeScaler roids;
            try
            {
                roids = hd.createROIDeScaler(entdec, pl, decSpec);
            }
            catch (ArgumentException e)
            {
                throw new InvalidOperationException("Cannot instantiate roi de-scaler.", e);
            }

            // **** Dequantizer ****
            Dequantizer deq;
            try
            {
                deq = hd.createDequantizer(roids, depth, decSpec);
            }
            catch (ArgumentException e)
            {
                throw new InvalidOperationException("Cannot instantiate dequantizer.", e);
            }

            // **** Inverse wavelet transform ***
            InverseWT invWT;
            try
            {
                // full page inverse wavelet transform
                invWT = InverseWT.createInstance(deq, decSpec);
            }
            catch (ArgumentException e)
            {
                throw new InvalidOperationException("Cannot instantiate inverse wavelet transform.", e);
            }

            int res = breader.ImgRes;
            invWT.ImgResLevel = res;

            // **** Data converter **** (after inverse transform module)
            ImgDataConverter converter = new ImgDataConverter(invWT, 0);

            // **** Inverse component transformation **** 
            InvCompTransf ictransf = new InvCompTransf(converter, decSpec, depth, pl);

            // **** Color space mapping ****
            BlkImgDataSrc color;
            if (ff.JP2FFUsed && pl.getParameter("nocolorspace").Equals("off"))
            {
                try
                {
                    ColorSpace csMap = new ColorSpace(in_stream, hd, pl);
                    BlkImgDataSrc channels = hd.createChannelDefinitionMapper(ictransf, csMap);
                    BlkImgDataSrc resampled = hd.createResampler(channels, csMap);
                    // the palette mapper only handles palettized (1 or 3 component) images
                    BlkImgDataSrc palettized = csMap.Palettized ? hd.createPalettizedColorSpaceMapper(resampled, csMap) : resampled;
                    color = hd.createColorSpaceMapper(palettized, csMap);
                }
                catch (ArgumentException e)
                {
                    throw new InvalidOperationException("Could not instantiate ICC profiler.", e);
                }
                catch (ColorSpaceException e)
                {
                    throw new InvalidOperationException("Error processing ColorSpace information.", e);
                }
            }
            else
            {
                // Skip colorspace mapping
                color = ictransf;
            }

            // This is the last image in the decoding chain and should be
            // assigned by the last transformation:
            BlkImgDataSrc decodedImage = color;
            if (color == null)
            {
                decodedImage = ictransf;
            }

            int numComps = decodedImage.NumComps;
            int imgWidth = decodedImage.ImgWidth;
            int imgHeight = decodedImage.ImgHeight;

            bool hasAlpha = !stripAlpha && (numComps == 2 || numComps >= 4);
            int outputBytesPerPixel = hasAlpha ? 4 : 3;
            var pixelFormat = hasAlpha ? PixelFormat.Format32bppArgb : PixelFormat.Format24bppRgb;

            // source component for each output byte (B,G,R[,A]); gray is replicated and a 5th (bump) component is dropped
            int[] slotComp = numComps switch
            {
                1 => [0, 0, 0],
                2 => [0, 0, 0, 1],
                3 => [2, 1, 0],
                4 or 5 => [2, 1, 0, 3],
                _ => throw new ApplicationException("Unsupported PixelFormat.  " + numComps + " components."),
            };

            // **** Copy to Bitmap ****
            Bitmap dst = new Bitmap(imgWidth, imgHeight, pixelFormat);

            // lock the whole bitmap once; locking partial regions of a 32bpp bitmap corrupts data on libgdiplus
            BitmapData dstdata = dst.LockBits(new Rectangle(0, 0, imgWidth, imgHeight), ImageLockMode.WriteOnly, pixelFormat);

            Coord numTiles = decodedImage.getNumTilesCoord();

            DataBlkInt[] db = new DataBlkInt[numComps];
            int[] ls = new int[numComps];
            int[] mv = new int[numComps];
            int[] fb = new int[numComps];
            int[] k = new int[numComps];
            int[] tmp = new int[numComps];
            for (int i = 0; i < numComps; i++)
            {
                db[i] = new DataBlkInt();
                int nomRangeBits = decodedImage.getNomRangeBits(i);
                ls[i] = 1 << (nomRangeBits - 1);
                mv[i] = (1 << nomRangeBits) - 1;
                fb[i] = decodedImage.getFixedPoint(i);
            }

            int tIdx = 0;

            for (int y = 0; y < numTiles.y; y++)
            {
                // Loop on horizontal tiles
                for (int x = 0; x < numTiles.x; x++, tIdx++)
                {
                    decodedImage.setTile(x, y);

                    int height = decodedImage.getTileCompHeight(tIdx, 0);
                    int width = decodedImage.getTileCompWidth(tIdx, 0);

                    int tOffx = decodedImage.getCompULX(0)
                                - (int)Math.Ceiling(decodedImage.ImgULX / (double)decodedImage.getCompSubsX(0));

                    int tOffy = decodedImage.getCompULY(0)
                                - (int)Math.Ceiling(decodedImage.ImgULY / (double)decodedImage.getCompSubsY(0));

                    if (tOffx < 0 || tOffy < 0 || tOffx + width > imgWidth || tOffy + height > imgHeight)
                    {
                        dst.UnlockBits(dstdata);
                        throw new InvalidOperationException($"Tile {x},{y} ({tOffx},{tOffy} {width}x{height}) outside of image {imgWidth}x{imgHeight}.");
                    }

                    byte[] rowvalues = new byte[width * outputBytesPerPixel];

                    for (int l = 0; l < height; l++)
                    {
                        // components must be requested in ascending order: requesting a component that
                        // bypasses the inverse component transform (e.g. alpha) first corrupts its first row
                        for (int i = 0; i < numComps; i++)
                        {
                            db[i].ulx = 0;
                            db[i].uly = l;
                            db[i].w = width;
                            db[i].h = 1;
                            decodedImage.getInternCompData(db[i], i);
                            k[i] = db[i].offset + width - 1;
                        }

                        for (int i = width - 1; i >= 0; i--)
                        {
                            for (int j = 0; j < numComps; j++)
                            {
                                int v = (db[j].data_array[k[j]--] >> fb[j]) + ls[j];
                                tmp[j] = (v < 0) ? 0 : ((v > mv[j]) ? mv[j] : v);
                            }
                            int offset = i * outputBytesPerPixel;
                            for (int s = 0; s < outputBytesPerPixel; s++)
                                rowvalues[offset + s] = (byte)tmp[slotComp[s]];
                        }

                        IntPtr ptr = dstdata.Scan0 + (tOffy + l) * dstdata.Stride + tOffx * outputBytesPerPixel;
                        System.Runtime.InteropServices.Marshal.Copy(rowvalues, 0, ptr, rowvalues.Length);
                    }
                }
            }

            dst.UnlockBits(dstdata);
            return dst;
        }

        public static List<int> GetLayerBoundaries(Stream stream, ParameterList parameters = null)
        {
            RandomAccessIO in_stream = new ISRandomAccessIO(stream);

            var pl = parameters ?? new ParameterList(GetDefaultDecoderParameterList(decoder_pinfo));

            // **** File Format ****
            // If the codestream is wrapped in the jp2 fileformat, Read the
            // file format wrapper
            FileFormatReader ff = new FileFormatReader(in_stream);
            ff.readFileFormat();
            if (ff.JP2FFUsed)
            {
                in_stream.seek(ff.FirstCodeStreamPos);
            }

            // +----------------------------+
            // | Instantiate decoding chain |
            // +----------------------------+

            // **** Header decoder ****
            // Instantiate header decoder and read main header 
            HeaderInfo hi = new HeaderInfo();
            HeaderDecoder hd;
            try
            {
                hd = new HeaderDecoder(in_stream, pl, hi);
            }
            catch (EndOfStreamException e)
            {
                throw new ArgumentException("Codestream too short or bad header, unable to decode.", e);
            }

            int nCompCod = hd.NumComps;
            int nTiles = hi.sizValue.NumTiles;
            DecoderSpecs decSpec = hd.DecoderSpecs;

            // Get demixed bitdepths
            int[] depth = new int[nCompCod];
            for (int i = 0; i < nCompCod; i++)
            {
                depth[i] = hd.getOriginalBitDepth(i);
            }

            // **** Bit stream reader ****
            BitstreamReaderAgent breader;
            try
            {
                breader = BitstreamReaderAgent.createInstance(in_stream, hd, pl, decSpec, false, hi);
            }
            catch (IOException e)
            {
                throw new ArgumentException("Error while reading bit stream header or parsing packets.", e);
            }
            catch (ArgumentException e)
            {
                throw new ArgumentException("Cannot instantiate bit stream reader.", e);
            }

            breader.setTile(0, 0);

            return ((FileBitstreamReaderAgent)breader).layerStarts;
        }

        #endregion

        #region Validation

        /// <summary>Maximum number of bytes accepted after the EOC marker</summary>
        public const int MaxBytesAfterEOC = 64;

        /// <summary> Checks that the data is a structurally valid JPEG 2000 image, either a raw
        /// codestream (j2c/j2k) or a JP2 file, without decoding it. The JP2 boxes, the main header and
        /// the tile-part headers are parsed and checked; the packet data is not.
        /// </summary>
        /// <param name="data">The image data</param>
        /// <param name="truncatedIsError">If true, data that ends before the EOC marker of the
        /// codestream is an error. Otherwise it is accepted (it can be decoded) as long as the JP2
        /// header, if any, and the main header of the codestream are complete.
        /// </param>
        /// <returns>null if the data is valid, otherwise a short description of the problem</returns>
        public static string Validate(byte[] data, bool truncatedIsError = false)
        {
            if (data == null)
                return "no data";
            return Validate(new ReadOnlySpan<byte>(data), truncatedIsError);
        }

        /// <summary> Checks that the data is a structurally valid JPEG 2000 image, either a raw
        /// codestream (j2c/j2k) or a JP2 file, without decoding it. See <see cref="Validate(byte[], bool)"/>.
        /// </summary>
        public static string Validate(ReadOnlySpan<byte> data, bool truncatedIsError = false)
        {
            if (data.Length < 2)
                return "too short";
            if (data[0] == 0xFF && data[1] == 0x4F)
                return ValidateCodestream(data, truncatedIsError, false, 0);
            if (data.Length >= 8 && BinaryPrimitives.ReadUInt32BigEndian(data) == 12
                && BinaryPrimitives.ReadUInt32BigEndian(data.Slice(4)) == FileFormatBoxes.JP2_SIGNATURE_BOX)
                return ValidateJP2(data, truncatedIsError);
            return "not JPEG 2000 data";
        }

        /// <summary> Checks the boxes of a JP2 file and then the codestream in its contiguous
        /// codestream box. Bytes after that box count as bytes after the EOC marker.
        /// </summary>
        private static string ValidateJP2(ReadOnlySpan<byte> data, bool truncatedIsError)
        {
            int len = data.Length;
            if (len < 12)
                return "truncated JP2 header";
            if (BinaryPrimitives.ReadUInt32BigEndian(data.Slice(8)) != 0x0D0A870A)
                return "bad JP2 signature";

            bool hasHeader = false;
            uint width = 0, height = 0;
            int numComps = 0;
            long pos = 12;
            for (int box = 0; ; box++)
            {
                if (pos >= len)
                    return "no codestream box";
                if (pos + 8 > len)
                    return "truncated JP2 header";
                long lbox = BinaryPrimitives.ReadUInt32BigEndian(data.Slice((int)pos));
                int tbox = BinaryPrimitives.ReadInt32BigEndian(data.Slice((int)pos + 4));
                int hlen = 8;
                bool toEnd = false;
                if (lbox == 1)
                {
                    if (pos + 16 > len)
                        return "truncated JP2 header";
                    ulong xlbox = BinaryPrimitives.ReadUInt64BigEndian(data.Slice((int)pos + 8));
                    if (xlbox < 16 || xlbox > int.MaxValue)
                        return "bad box length at " + pos;
                    lbox = (long)xlbox;
                    hlen = 16;
                }
                else if (lbox == 0)
                {
                    lbox = len - pos;
                    toEnd = true;
                }
                else if (lbox < 8)
                {
                    return "bad box length at " + pos;
                }
                long end = pos + lbox;
                bool boxTruncated = end > len;

                if (box == 0 && tbox != FileFormatBoxes.FILE_TYPE_BOX)
                    return "no file type box";

                if (tbox == FileFormatBoxes.CONTIGUOUS_CODESTREAM_BOX)
                {
                    if (!hasHeader)
                        return "no JP2 header box";
                    ReadOnlySpan<byte> cs = data.Slice((int)pos + hlen, (int)(Math.Min(end, len) - pos - hlen));

                    // The image header must describe the codestream
                    if (cs.Length >= 42 && cs[0] == 0xFF && cs[1] == 0x4F && cs[2] == 0xFF && cs[3] == 0x51)
                    {
                        uint w = BinaryPrimitives.ReadUInt32BigEndian(cs.Slice(8)) - BinaryPrimitives.ReadUInt32BigEndian(cs.Slice(16));
                        uint h = BinaryPrimitives.ReadUInt32BigEndian(cs.Slice(12)) - BinaryPrimitives.ReadUInt32BigEndian(cs.Slice(20));
                        if (w != width || h != height || BinaryPrimitives.ReadUInt16BigEndian(cs.Slice(40)) != numComps)
                            return "JP2 header does not match codestream";
                    }

                    long after = boxTruncated ? 0 : len - end;
                    if (after > MaxBytesAfterEOC)
                        return after + " bytes after codestream";
                    if (cs.Length < 2 || cs[0] != 0xFF || cs[1] != 0x4F)
                        return cs.Length < 2 ? "truncated JP2 header" : "no SOC marker";
                    return ValidateCodestream(cs, truncatedIsError, !boxTruncated && !toEnd, (int)after);
                }

                // All other boxes come before the codestream and must be complete
                if (boxTruncated)
                    return "truncated JP2 header";
                ReadOnlySpan<byte> content = data.Slice((int)pos + hlen, (int)(end - pos - hlen));
                if (tbox == FileFormatBoxes.FILE_TYPE_BOX)
                {
                    if (box != 0 || content.Length < 8 || (content.Length - 8) % 4 != 0)
                        return "bad file type box";
                    bool jp2 = BinaryPrimitives.ReadInt32BigEndian(content) == FileFormatBoxes.FT_BR;
                    for (int i = 8; i < content.Length && !jp2; i += 4)
                        jp2 = BinaryPrimitives.ReadInt32BigEndian(content.Slice(i)) == FileFormatBoxes.FT_BR;
                    if (!jp2)
                        return "not a JP2 file type";
                }
                else if (tbox == FileFormatBoxes.JP2_HEADER_BOX)
                {
                    // The image header box comes first, followed by other boxes that must fit
                    if (hasHeader)
                        return "more than one JP2 header box";
                    if (content.Length < 22 || BinaryPrimitives.ReadUInt32BigEndian(content) != 22
                        || BinaryPrimitives.ReadInt32BigEndian(content.Slice(4)) != FileFormatBoxes.IMAGE_HEADER_BOX)
                        return "bad image header box";
                    height = BinaryPrimitives.ReadUInt32BigEndian(content.Slice(8));
                    width = BinaryPrimitives.ReadUInt32BigEndian(content.Slice(12));
                    numComps = BinaryPrimitives.ReadUInt16BigEndian(content.Slice(16));
                    if (width == 0 || height == 0 || numComps == 0 || content[19] != FileFormatBoxes.IMB_C)
                        return "bad image header box";
                    for (int i = 22; i < content.Length; )
                    {
                        if (i + 8 > content.Length)
                            return "bad JP2 header box";
                        uint l = BinaryPrimitives.ReadUInt32BigEndian(content.Slice(i));
                        if (l < 8 || l > content.Length - i)
                            return "bad JP2 header box";
                        i += (int)l;
                    }
                    hasHeader = true;
                }
                pos = end;
            }
        }

        /// <summary> Checks a raw codestream.
        /// </summary>
        /// <param name="complete">The data is known to be complete (its length was given by a JP2
        /// box), so running out of data is an error and not a truncation</param>
        /// <param name="bytesAfter">Number of bytes following the data, counted with the bytes after EOC</param>
        private static string ValidateCodestream(ReadOnlySpan<byte> data, bool truncatedIsError, bool complete, int bytesAfter)
        {
            int len = data.Length;
            // Result for data that ends early
            string Truncated(string msg) => truncatedIsError || complete ? msg : null;

            // **** SIZ, must be the first marker segment ****
            if (len < 6)
                return "truncated main header";
            if (data[2] != 0xFF || data[3] != 0x51)
                return "no SIZ marker";
            int lsiz = BinaryPrimitives.ReadUInt16BigEndian(data.Slice(4));
            if (6 + 36 > len || 4 + lsiz > len)
                return "truncated main header";
            ReadOnlySpan<byte> siz = data.Slice(4, lsiz);
            if (lsiz < 41)
                return "bad SIZ length";

            ulong xsiz = BinaryPrimitives.ReadUInt32BigEndian(siz.Slice(4));
            ulong ysiz = BinaryPrimitives.ReadUInt32BigEndian(siz.Slice(8));
            ulong xosiz = BinaryPrimitives.ReadUInt32BigEndian(siz.Slice(12));
            ulong yosiz = BinaryPrimitives.ReadUInt32BigEndian(siz.Slice(16));
            ulong xtsiz = BinaryPrimitives.ReadUInt32BigEndian(siz.Slice(20));
            ulong ytsiz = BinaryPrimitives.ReadUInt32BigEndian(siz.Slice(24));
            ulong xtosiz = BinaryPrimitives.ReadUInt32BigEndian(siz.Slice(28));
            ulong ytosiz = BinaryPrimitives.ReadUInt32BigEndian(siz.Slice(32));
            int csiz = BinaryPrimitives.ReadUInt16BigEndian(siz.Slice(36));

            if (xsiz <= xosiz || ysiz <= yosiz)
                return "bad image size";
            if (xtsiz == 0 || ytsiz == 0 || xtosiz > xosiz || ytosiz > yosiz || xtosiz + xtsiz <= xosiz || ytosiz + ytsiz <= yosiz)
                return "bad tile size";
            if (csiz < 1 || csiz > 16384 || lsiz != 38 + 3 * csiz)
                return "bad number of components";
            for (int c = 0, o = 38; c < csiz; c++, o += 3)
            {
                if ((siz[o] & 0x7F) > 37 || siz[o + 1] == 0 || siz[o + 2] == 0)
                    return "bad component " + c;
            }

            ulong numTilesL = ((xsiz - xtosiz + xtsiz - 1) / xtsiz) * ((ysiz - ytosiz + ytsiz - 1) / ytsiz);
            if (numTilesL > 65535)
                return "too many tiles";
            int numTiles = (int)numTilesL;

            // **** Rest of the main header, up to the first SOT ****
            bool hasCOD = false;
            bool hasQCD = false;
            int pos = 4 + lsiz;
            while (true)
            {
                if (pos + 2 > len)
                    return "truncated main header";
                if (data[pos] != 0xFF)
                    return "bad marker at " + pos;
                int m = data[pos + 1];
                if (m == 0x90) // SOT
                    break;
                if (m >= 0x30 && m <= 0x3F) // reserved markers without segment
                {
                    pos += 2;
                    continue;
                }
                if (m == 0x4F || m == 0x51 || m == 0x93 || m == 0xD9 || m == 0x91 || m == 0x92 || m == 0x58 || m == 0x61)
                    return "unexpected marker FF" + m.ToString("X2") + " in main header";
                if (pos + 4 > len)
                    return "truncated main header";
                int lseg = BinaryPrimitives.ReadUInt16BigEndian(data.Slice(pos + 2));
                if (lseg < 2)
                    return "bad marker length at " + pos;
                if (pos + 2 + lseg > len)
                    return "truncated main header";
                if (m == 0x52) // COD
                {
                    string err = CheckCOD(data.Slice(pos + 2, lseg));
                    if (err != null)
                        return err;
                    hasCOD = true;
                }
                else if (m == 0x5C) // QCD
                {
                    if (lseg < 4)
                        return "bad QCD";
                    hasQCD = true;
                }
                pos += 2 + lseg;
            }
            if (!hasCOD)
                return "no COD marker";
            if (!hasQCD)
                return "no QCD marker";

            // **** Tile-parts ****
            // Number of tile-parts read per tile, to check their order and that all tiles are present
            byte[] rented = null;
            Span<byte> tileParts = numTiles <= 1024
                ? stackalloc byte[numTiles]
                : (rented = System.Buffers.ArrayPool<byte>.Shared.Rent(numTiles)).AsSpan(0, numTiles);
            try
            {
                tileParts.Clear();
                while (true)
                {
                    if (pos >= len)
                        return Truncated("truncated, no EOC");
                    if (pos + 2 > len)
                        return Truncated("truncated");
                    if (data[pos] != 0xFF)
                        return "bad marker at " + pos;
                    int m = data[pos + 1];
                    if (m == 0xD9) // EOC
                    {
                        int trailing = len - pos - 2 + bytesAfter;
                        if (trailing > MaxBytesAfterEOC)
                            return trailing + " bytes after EOC";
                        for (int t = 0; t < numTiles; t++)
                        {
                            if (tileParts[t] == 0)
                                return "missing tile " + t;
                        }
                        return null;
                    }
                    if (m != 0x90)
                        return "expected SOT at " + pos + ", found FF" + m.ToString("X2");
                    if (pos + 12 > len)
                        return Truncated("truncated");

                    int lsot = BinaryPrimitives.ReadUInt16BigEndian(data.Slice(pos + 2));
                    int isot = BinaryPrimitives.ReadUInt16BigEndian(data.Slice(pos + 4));
                    uint psot = BinaryPrimitives.ReadUInt32BigEndian(data.Slice(pos + 6));
                    int tpsot = data[pos + 10];
                    int tnsot = data[pos + 11];
                    if (lsot != 10)
                        return "bad SOT length at " + pos;
                    if (isot >= numTiles)
                        return "bad tile index " + isot;
                    if (tpsot != tileParts[isot] || tpsot == 255 || (tnsot != 0 && tpsot >= tnsot))
                        return "out of order tile-part in tile " + isot;
                    tileParts[isot] = (byte)(tpsot + 1);

                    // End of the tile-part; Psot 0 means it extends up to the EOC marker
                    long end;
                    if (psot == 0)
                    {
                        end = -1;
                        int min = Math.Max(pos + 12, len - 2 - MaxBytesAfterEOC);
                        for (int e = len - 2; e >= min; e--)
                        {
                            if (data[e] == 0xFF && data[e + 1] == 0xD9)
                            {
                                end = e;
                                break;
                            }
                        }
                        if (end < 0)
                            end = (long)len + 1; // no EOC, data is truncated
                    }
                    else
                    {
                        if (psot < 14)
                            return "bad tile-part length at " + pos;
                        end = pos + (long)psot;
                    }

                    // Tile-part header, up to SOD
                    bool truncated = end > len;
                    int limit = truncated ? len : (int)end;
                    int hp = pos + 12;
                    while (true)
                    {
                        if (hp + 2 > limit)
                        {
                            if (truncated)
                                return Truncated("truncated in tile " + isot);
                            return "no SOD in tile-part at " + pos;
                        }
                        if (data[hp] != 0xFF)
                            return "bad marker at " + hp;
                        int tm = data[hp + 1];
                        if (tm == 0x93) // SOD
                            break;
                        if (tm >= 0x30 && tm <= 0x3F)
                        {
                            hp += 2;
                            continue;
                        }
                        if (tm != 0x52 && tm != 0x53 && tm != 0x5C && tm != 0x5D && tm != 0x5E && tm != 0x5F
                            && tm != 0x58 && tm != 0x61 && tm != 0x64)
                            return "unexpected marker FF" + tm.ToString("X2") + " in tile-part header";
                        if (hp + 4 > limit)
                            return truncated ? Truncated("truncated in tile " + isot) : "bad tile-part header at " + pos;
                        int lseg = BinaryPrimitives.ReadUInt16BigEndian(data.Slice(hp + 2));
                        if (lseg < 2)
                            return "bad marker length at " + hp;
                        if (hp + 2 + lseg > limit)
                            return truncated ? Truncated("truncated in tile " + isot) : "bad tile-part header at " + pos;
                        if (tm == 0x52)
                        {
                            string err = CheckCOD(data.Slice(hp + 2, lseg));
                            if (err != null)
                                return err;
                        }
                        hp += 2 + lseg;
                    }

                    if (truncated)
                        return Truncated("truncated in tile " + isot);
                    pos = (int)end;
                }
            }
            finally
            {
                if (rented != null)
                    System.Buffers.ArrayPool<byte>.Shared.Return(rented);
            }
        }

        /// <summary> Checks a COD marker segment (starting at Lcod) for values the decoder cannot handle.
        /// </summary>
        private static string CheckCOD(ReadOnlySpan<byte> cod)
        {
            if (cod.Length < 12)
                return "bad COD length";
            int progression = cod[3];
            int layers = BinaryPrimitives.ReadUInt16BigEndian(cod.Slice(4));
            int levels = cod[7];
            int cbw = cod[8] + 2;
            int cbh = cod[9] + 2;
            int transform = cod[11];
            if (progression > 4 || layers == 0 || levels > 32 || cbw > 10 || cbh > 10 || cbw + cbh > 12 || transform > 1)
                return "bad COD";
            if ((cod[2] & 0x01) != 0 && cod.Length < 12 + levels + 1) // user defined precincts
                return "bad COD length";
            return null;
        }

        #endregion

        #region Static Encoder Methods

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BlkImgDataSrc CreateEncodableSource(Stream stream)
        {
            return CreateEncodableSource(new[] { stream });
        }

        public static BlkImgDataSrc CreateEncodableSource(IEnumerable<Stream> streams)
        {
            if (streams == null)
            {
                throw new ArgumentNullException("streams");
            }

            var counter = 0;
            var ncomp = 0;
            var ppminput = false;
            var imageReaders = new List<ImgReader>();

            foreach (var stream in streams)
            {
                ++counter;
                var imgType = GetImageType(stream);

                switch (imgType)
                {
                    case "P5":
                        imageReaders.Add(new ImgReaderPGM(stream));
                        ncomp += 1;
                        break;
                    case "P6":
                        imageReaders.Add(new ImgReaderPPM(stream));
                        ncomp += 3;
                        ppminput = true;
                        break;
                    case "PG":
                        imageReaders.Add(new ImgReaderPGX(stream));
                        ncomp += 1;
                        break;
                    default:
                        throw new ArgumentOutOfRangeException("streams", "Invalid image type");
                }
            }

            if (ppminput && counter > 1)
            {
                error("With PPM input format only 1 input file can be specified", 2);
                return null;
            }

            BlkImgDataSrc imgsrc;

            // **** ImgDataJoiner (if needed) ****
            if (ppminput || ncomp == 1)
            {
                // Just one input
                imgsrc = imageReaders[0];
            }
            else
            {
                // More than one reader => join all readers into 1
                var imgcmpidxs = new int[ncomp];
                imgsrc = new ImgDataJoiner(imageReaders, imgcmpidxs);
            }

            return imgsrc;
        }

        /// <summary> Creates a source for the encoder from a bitmap, with 8 bits per component.
        /// Pixel formats with an alpha channel (including indexed formats whose palette has alpha)
        /// give 4 components (R, G, B, A), all others 3 components (R, G, B).
        /// </summary>
        public static BlkImgDataSrc CreateEncodableSource(Bitmap bitmap)
        {
            if (bitmap == null)
                throw new ArgumentNullException(nameof(bitmap));

            int w = bitmap.Width;
            int h = bitmap.Height;
            PixelFormat format = bitmap.PixelFormat;
            bool hasAlpha = Image.IsAlphaPixelFormat(format) || ((format & PixelFormat.Indexed) != 0 && PaletteHasAlpha(bitmap));
            int nc = hasAlpha ? 4 : 3;

            var comps = new int[nc][];
            for (int c = 0; c < nc; c++)
                comps[c] = new int[w * h];

            // GDI+ converts any pixel format to (non premultiplied) BGRA
            BitmapData data = bitmap.LockBits(new Rectangle(0, 0, w, h), ImageLockMode.ReadOnly, PixelFormat.Format32bppArgb);
            try
            {
                byte[] row = new byte[w * 4];
                for (int y = 0, k = 0; y < h; y++)
                {
                    System.Runtime.InteropServices.Marshal.Copy(data.Scan0 + y * data.Stride, row, 0, row.Length);
                    for (int x = 0, o = 0; x < w; x++, k++, o += 4)
                    {
                        comps[0][k] = row[o + 2];
                        comps[1][k] = row[o + 1];
                        comps[2][k] = row[o];
                        if (hasAlpha)
                            comps[3][k] = row[o + 3];
                    }
                }
            }
            finally
            {
                bitmap.UnlockBits(data);
            }

            return new PortableImageSource(w, h, nc, 8, new bool[nc], comps);
        }

        /// <summary> Whether an indexed bitmap's palette has transparent entries (the palette flags
        /// are not reliable, libgdiplus does not set them).
        /// </summary>
        private static bool PaletteHasAlpha(Bitmap bitmap)
        {
            foreach (var entry in bitmap.Palette.Entries)
            {
                if (entry.A != 255)
                    return true;
            }
            return false;
        }

        /// <summary> Encodes an image. A <see cref="Image"/> is encoded through
        /// <see cref="CreateEncodableSource(Bitmap)"/>, so with alpha if its pixel format has it; other
        /// objects need a registered <see cref="IImageCreator"/>.
        /// </summary>
        public static byte[] ToBytes(object imageObject, ParameterList parameters = null)
        {
            BlkImgDataSrc imgsrc;
            if (imageObject is Bitmap bitmap)
            {
                imgsrc = CreateEncodableSource(bitmap);
            }
            else if (imageObject is Image image)
            {
                using var converted = new Bitmap(image);
                imgsrc = CreateEncodableSource(converted);
            }
            else
            {
                imgsrc = ImageFactory.ToPortableImageSource(imageObject);
                if (imgsrc == null)
                    throw new ArgumentException("No image source for this object type; register an IImageCreator or use CreateEncodableSource", nameof(imageObject));
            }
            return ToBytes(imgsrc, parameters);
        }

        public static byte[] ToBytes(BlkImgDataSrc imgsrc, ParameterList parameters = null)
        {
            // Create parameter list using defaults
            ParameterList pl = parameters ?? new ParameterList(GetDefaultEncoderParameterList(encoder_pinfo));;

            bool useFileFormat = false;
            bool pphTile = false;
            bool pphMain = false;
            bool tempSop = false;
            bool tempEph = false;

            // **** Get general parameters ****

            if (pl.getParameter("file_format").Equals("on"))
            {
                useFileFormat = true;
                if (pl.getParameter("rate") != null && pl.getFloatParameter("rate") != pl.getFloatParameter("rate"))
                {
                    warning("Specified bit-rate applies only on the codestream but not on the whole file.");
                }
            }

            if (pl.getParameter("tiles") == null)
            {
                error("No tiles option specified", 2);
                return null;
            }

            if (pl.getParameter("pph_tile").Equals("on"))
            {
                pphTile = true;

                if (pl.getParameter("Psop").Equals("off"))
                {
                    pl["Psop"] = "on";
                    tempSop = true;
                }
                if (pl.getParameter("Peph").Equals("off"))
                {
                    pl["Peph"] = "on";
                    tempEph = true;
                }
            }

            if (pl.getParameter("pph_main").Equals("on"))
            {
                pphMain = true;

                if (pl.getParameter("Psop").Equals("off"))
                {
                    pl["Psop"] = "on";
                    tempSop = true;
                }
                if (pl.getParameter("Peph").Equals("off"))
                {
                    pl["Peph"] = "on";
                    tempEph = true;
                }
            }

            if (pphTile && pphMain) error("Can't have packed packet headers in both main and" + " tile headers", 2);

            if (pl.getBooleanParameter("lossless") && pl.getParameter("rate") != null
                && pl.getFloatParameter("rate") != pl.getFloatParameter("rate")) throw new ArgumentException("Cannot use '-rate' and " + "'-lossless' option at " + " the same time.");

            if (pl.getParameter("rate") == null)
            {
                error("Target bitrate not specified", 2);
                return null;
            }
            float rate;
            try
            {
                rate = pl.getFloatParameter("rate");
                if (rate == -1)
                {
                    rate = float.MaxValue;
                }
            }
            catch (FormatException)
            {
                error("Invalid value in 'rate' option: " + pl.getParameter("rate"), 2);
                return null;
            }
            int pktspertp;
            try
            {
                pktspertp = pl.getIntParameter("tile_parts");
                if (pktspertp != 0)
                {
                    if (pl.getParameter("Psop").Equals("off"))
                    {
                        pl["Psop"] = "on";
                        tempSop = true;
                    }
                    if (pl.getParameter("Peph").Equals("off"))
                    {
                        pl["Peph"] = "on";
                        tempEph = true;
                    }
                }
            }
            catch (FormatException)
            {
                error("Invalid value in 'tile_parts' option: " + pl.getParameter("tile_parts"), 2);
                return null;
            }

            // **** ImgReader ****
            var ncomp = imgsrc.NumComps;
            var ppminput = imgsrc.NumComps > 1;

            // **** Tiler ****
            // get nominal tile dimensions
            SupportClass.StreamTokenizerSupport stok =
                new SupportClass.StreamTokenizerSupport(new StringReader(pl.getParameter("tiles")));
            stok.EOLIsSignificant(false);

            stok.NextToken();
            if (stok.ttype != SupportClass.StreamTokenizerSupport.TT_NUMBER)
            {
                error("An error occurred while parsing the tiles option: " + pl.getParameter("tiles"), 2);
                return null;
            }
            var tw = (int)stok.nval;
            stok.NextToken();
            if (stok.ttype != SupportClass.StreamTokenizerSupport.TT_NUMBER)
            {
                error("An error occurred while parsing the tiles option: " + pl.getParameter("tiles"), 2);
                return null;
            }
            var th = (int)stok.nval;

            // Get image reference point
            var refs = pl.getParameter("ref").Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            int refx;
            int refy;
            try
            {
                refx = Int32.Parse(refs[0]);
                refy = Int32.Parse(refs[1]);
            }
            catch (IndexOutOfRangeException)
            {
                throw new ArgumentException("Error while parsing 'ref' " + "option");
            }
            catch (FormatException)
            {
                throw new ArgumentException("Invalid number type in " + "'ref' option");
            }
            if (refx < 0 || refy < 0)
            {
                throw new ArgumentException("Invalid value in 'ref' " + "option ");
            }

            // Get tiling reference point
            var trefs = pl.getParameter("tref").Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            int trefx;
            int trefy;
            try
            {
                trefx = Int32.Parse(trefs[0]);
                trefy = Int32.Parse(trefs[1]);
            }
            catch (IndexOutOfRangeException)
            {
                throw new ArgumentException("Error while parsing 'tref' " + "option");
            }
            catch (FormatException)
            {
                throw new ArgumentException("Invalid number type in " + "'tref' option");
            }
            if (trefx < 0 || trefy < 0 || trefx > refx || trefy > refy)
            {
                throw new ArgumentException("Invalid value in 'tref' " + "option ");
            }

            // Instantiate tiler
            Tiler imgtiler;
            try
            {
                imgtiler = new Tiler(imgsrc, refx, refy, trefx, trefy, tw, th);
            }
            catch (ArgumentException e)
            {
                error("Could not tile image" + ((e.Message != null) ? (":\n" + e.Message) : ""), 2);
                return null;
            }
            int ntiles = imgtiler.getNumTiles();

            // **** Encoder specifications ****
            var encSpec = new EncoderSpecs(ntiles, ncomp, imgsrc, pl);

            // **** Component transformation ****
            if (ppminput && pl.getParameter("Mct") != null && pl.getParameter("Mct").Equals("off"))
            {
                FacilityManager.getMsgLogger()
                    .printmsg(
                        MsgLogger_Fields.WARNING,
                        "Input image is RGB and no color transform has "
                        + "been specified. Compression performance and "
                        + "image quality might be greatly degraded. Use "
                        + "the 'Mct' option to specify a color transform");
            }
            ForwCompTransf fctransf;
            try
            {
                fctransf = new ForwCompTransf(imgtiler, encSpec);
            }
            catch (ArgumentException e)
            {
                error(
                    "Could not instantiate forward component " + "transformation"
                    + ((e.Message != null) ? (":\n" + e.Message) : ""),
                    2);
                return null;
            }

            // **** ImgDataConverter ****
            var converter = new ImgDataConverter(fctransf);


            // **** ForwardWT ****
            ForwardWT dwt;
            try
            {
                dwt = ForwardWT.createInstance(converter, pl, encSpec);
            }
            catch (ArgumentException e)
            {
                error("Could not instantiate wavelet transform" + ((e.Message != null) ? (":\n" + e.Message) : ""), 2);
                return null;
            }

            // **** Quantizer ****
            Quantizer quant;
            try
            {
                quant = Quantizer.createInstance(dwt, encSpec);
            }
            catch (ArgumentException e)
            {
                error("Could not instantiate quantizer" + ((e.Message != null) ? (":\n" + e.Message) : ""), 2);
                return null;
            }

            // **** ROIScaler ****
            ROIScaler rois;
            try
            {
                rois = ROIScaler.createInstance(quant, pl, encSpec);
            }
            catch (ArgumentException e)
            {
                error("Could not instantiate ROI scaler" + ((e.Message != null) ? (":\n" + e.Message) : ""), 2);
                return null;
            }

            // **** EntropyCoder ****
            EntropyCoder ecoder;
            try
            {
                ecoder = EntropyCoder.createInstance(
                    rois,
                    pl,
                    encSpec.cblks,
                    encSpec.pss,
                    encSpec.bms,
                    encSpec.mqrs,
                    encSpec.rts,
                    encSpec.css,
                    encSpec.sss,
                    encSpec.lcs,
                    encSpec.tts);
            }
            catch (ArgumentException e)
            {
                error("Could not instantiate entropy coder" + ((e.Message != null) ? (":\n" + e.Message) : ""), 2);
                return null;
            }

            // **** CodestreamWriter ****
            using (var outStream = new MemoryStream())
            {
                CodestreamWriter bwriter;
                try
                {
                    // Rely on rate allocator to limit amount of data
                    bwriter = new FileCodestreamWriter(outStream, Int32.MaxValue);
                }
                catch (IOException e)
                {
                    error("Could not open output file" + ((e.Message != null) ? (":\n" + e.Message) : ""), 2);
                    return null;
                }

                // **** Rate allocator ****
                PostCompRateAllocator ralloc;
                try
                {
                    ralloc = PostCompRateAllocator.createInstance(ecoder, pl, rate, bwriter, encSpec);
                }
                catch (ArgumentException e)
                {
                    error("Could not instantiate rate allocator" + ((e.Message != null) ? (":\n" + e.Message) : ""), 2);
                    return null;
                }

                // **** HeaderEncoder ****
                var imsigned = Enumerable.Repeat(false, ncomp).ToArray();   // TODO Consider supporting signed components.
                var headenc = new HeaderEncoder(imgsrc, imsigned, dwt, imgtiler, encSpec, rois, ralloc, pl);
                ralloc.HeaderEncoder = headenc;

                // **** Write header to be able to estimate header overhead ****
                headenc.encodeMainHeader();

                // **** Initialize rate allocator, with proper header
                // overhead. This will also encode all the data ****
                ralloc.initialize();

                // **** Write header (final) ****
                headenc.reset();
                headenc.encodeMainHeader();

                // Insert header into the codestream
                bwriter.commitBitstreamHeader(headenc);

                // **** Now do the rate-allocation and write result ****
                ralloc.runAndWrite();

                // **** Done ****
                bwriter.close();

                // **** Calculate file length ****
                int fileLength = bwriter.Length;

                // **** Tile-parts and packed packet headers ****
                if (pktspertp > 0 || pphTile || pphMain)
                {
                    try
                    {
                        CodestreamManipulator cm = new CodestreamManipulator(
                            outStream,
                            ntiles,
                            pktspertp,
                            pphMain,
                            pphTile,
                            tempSop,
                            tempEph);
                        fileLength += cm.doCodestreamManipulation();
                        // The codestream is rewritten in place and may have become shorter (the
                        // temporary SOP and EPH markers are removed): drop what is left after it
                        outStream.SetLength(fileLength);
                        //String res="";
                        if (pktspertp > 0)
                        {
                            FacilityManager.getMsgLogger()
                                .println(
                                    "Created tile-parts " + "containing at most " + pktspertp + " packets per tile.",
                                    4,
                                    6);
                        }
                        if (pphTile)
                        {
                            FacilityManager.getMsgLogger().println("Moved packet headers " + "to tile headers", 4, 6);
                        }
                        if (pphMain)
                        {
                            FacilityManager.getMsgLogger().println("Moved packet headers " + "to main header", 4, 6);
                        }
                    }
                    catch (IOException e)
                    {
                        error(
                            "Error while creating tileparts or packed packet" + " headers"
                            + ((e.Message != null) ? (":\n" + e.Message) : ""),
                            2);
                        return null;
                    }
                }

                // **** File Format ****
                if (useFileFormat)
                {
                    try
                    {
                        int nc = imgsrc.NumComps;
                        int[] bpc = new int[nc];
                        for (int comp = 0; comp < nc; comp++)
                        {
                            bpc[comp] = imgsrc.getNomRangeBits(comp);
                        }

                        outStream.Seek(0, SeekOrigin.Begin);
                        var ffw = new FileFormatWriter(
                            outStream,
                            imgsrc.ImgHeight,
                            imgsrc.ImgWidth,
                            nc,
                            bpc,
                            fileLength);
                        fileLength += ffw.writeFileFormat();
                    }
                    catch (IOException e)
                    {
                        throw new InvalidOperationException("Error while writing JP2 file format: " + e.Message);
                    }
                }

                // **** Close image readers ***
                imgsrc.close();

                return outStream.ToArray();
            }
        }

        #endregion

        #region Default Parameter Loaders

        public static ParameterList GetDefaultDecoderParameterList()
        {
            return GetDefaultDecoderParameterList(decoder_pinfo);
        }

        private static ParameterList CacheDefaultDecoder = null;


        public static ParameterList GetDefaultDecoderParameterList(string[][] pinfo)
        {
            if (CacheDefaultDecoder != null)
                return CacheDefaultDecoder;

            ParameterList pl = new ParameterList();
            string[][] str;

            str = BitstreamReaderAgent.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = EntropyDecoder.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = ROIDeScaler.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = Dequantizer.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = InvCompTransf.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = HeaderDecoder.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = ICCProfiler.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = pinfo ?? decoder_pinfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            CacheDefaultDecoder = pl;
            return pl;
        }

        public static ParameterList GetDefaultEncoderParameterList(string[][] pinfo)
        {
            ParameterList pl =  new ParameterList();
            string[][] str;

            str = pinfo ?? encoder_pinfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = ForwCompTransf.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = AnWTFilter.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = ForwardWT.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = Quantizer.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = ROIScaler.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = EntropyCoder.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = HeaderEncoder.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = PostCompRateAllocator.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            str = PktEncoder.ParameterInfo;
            if (str != null) for (int i = 0; i < str.Length; i++) pl[str[i][0]] = str[i][3];

            return pl;
        }

        #endregion

        #region Decoder Parameters

        private static String[][] decoder_pinfo =
            {
                new string[]
                    {
                        "u", "[on|off]",
                        "Prints usage information. "
                        + "If specified all other arguments (except 'v') are ignored",
                        "off"
                    },
                new string[]
                    {
                        "v", "[on|off]", "Prints version and copyright information",
                        "off"
                    },
                new string[]
                    {
                        "verbose", "[on|off]",
                        "Prints information about the decoded codestream", "on"
                    },
                new string[]
                    {
                        "pfile", "<filename>",
                        "Loads the arguments from the specified file. Arguments that are "
                        + "specified on the command line override the ones from the file.\n"
                        + "The arguments file is a simple text file with one argument per "
                        + "line of the following form:\n"
                        + "  <argument name>=<argument value>\n"
                        + "If the argument is of boolean type (i.e. its presence turns a "
                        + "feature on), then the 'on' value turns it on, while the 'off' "
                        + "value turns it off. The argument name does not include the '-' "
                        + "or '+' character. Long lines can be broken into several lines "
                        + "by terminating them with '\\'. Lines starting with '#' are "
                        + "considered as comments. This option is not recursive: any 'pfile' "
                        + "argument appearing in the file is ignored.",
                        null
                    },
                new string[]
                    {
                        "res", "<resolution level index>",
                        "The resolution level at which to reconstruct the image "
                        + " (0 means the lowest available resolution whereas the maximum "
                        + "resolution level corresponds to the original image resolution). "
                        + "If the given index"
                        + " is greater than the number of available resolution levels of the "
                        + "compressed image, the image is reconstructed at its highest "
                        + "resolution (among all tile-components). Note that this option"
                        + " affects only the inverse wavelet transform and not the number "
                        + " of bytes read by the codestream parser: this number of bytes "
                        + "depends only on options '-nbytes' or '-rate'.",
                        null
                    },
                new string[]
                    {
                        "i", "<filename or url>",
                        "The file containing the JPEG 2000 compressed data. This can be "
                        + "either a JPEG 2000 codestream or a JP2 file containing a "
                        + "JPEG 2000 "
                        + "codestream. In the latter case the first codestream in the file "
                        + "will be decoded. If an URL is specified (e.g., http://...) "
                        + "the data will be downloaded and cached in memory before decoding. "
                        + "This is intended for easy use in applets, but it is not a very "
                        + "efficient way of decoding network served data.",
                        null
                    },
                new string[]
                    {
                        "o", "<filename>",
                        "This is the name of the file to which the decompressed image "
                        + "is written. If no output filename is given, the image is "
                        + "displayed on the screen. "
                        + "Output file format is PGX by default. If the extension"
                        + " is '.pgm' then a PGM file is written as output, however this is "
                        + "only permitted if the component bitdepth does not exceed 8. If "
                        + "the extension is '.ppm' then a PPM file is written, however this "
                        + "is only permitted if there are 3 components and none of them has "
                        + "a bitdepth of more than 8. If there is more than 1 component, "
                        + "suffices '-1', '-2', '-3', ... are added to the file name, just "
                        + "before the extension, except for PPM files where all three "
                        + "components are written to the same file.",
                        null
                    },
                new string[]
                    {
                        "rate", "<decoding rate in bpp>",
                        "Specifies the decoding rate in bits per pixel (bpp) where the "
                        + "number of pixels is related to the image's original size (Note:"
                        + " this number is not affected by the '-res' option). If it is equal"
                        + "to -1, the whole codestream is decoded. "
                        + "The codestream is either parsed (default) or truncated depending "
                        + "the command line option '-parsing'. To specify the decoding "
                        + "rate in bytes, use '-nbytes' options instead.",
                        "-1"
                    },
                new string[]
                    {
                        "nbytes", "<decoding rate in bytes>",
                        "Specifies the decoding rate in bytes. "
                        + "The codestream is either parsed (default) or truncated depending "
                        + "the command line option '-parsing'. To specify the decoding "
                        + "rate in bits per pixel, use '-rate' options instead.",
                        "-1"
                    },
                new string[]
                    {
                        "parsing", null,
                        "Enable or not the parsing mode when decoding rate is specified "
                        + "('-nbytes' or '-rate' options). If it is false, the codestream "
                        + "is decoded as if it were truncated to the given rate. If it is "
                        + "true, the decoder creates, truncates and decodes a virtual layer"
                        + " progressive codestream with the same truncation points in each "
                        + "code-block.",
                        "on"
                    },
                new string[]
                    {
                        "ncb_quit", "<max number of code blocks>",
                        "Use the ncb and lbody quit conditions. If state information is "
                        + "found for more code blocks than is indicated with this option, "
                        + "the decoder "
                        + "will decode using only information found before that point. "
                        + "Using this otion implies that the 'rate' or 'nbyte' parameter "
                        + "is used to indicate the lbody parameter which is the number of "
                        + "packet body bytes the decoder will decode.",
                        "-1"
                    },
                new string[]
                    {
                        "l_quit", "<max number of layers>",
                        "Specifies the maximum number of layers to decode for any code-"
                        + "block",
                        "-1"
                    },
                new string[]
                    {
                        "m_quit", "<max number of bit planes>",
                        "Specifies the maximum number of bit planes to decode for any code"
                        + "-block",
                        "-1"
                    },
                new string[]
                    {
                        "poc_quit", null,
                        "Specifies the whether the decoder should only decode code-blocks "
                        + "included in the first progression order.",
                        "off"
                    },
                new string[]
                    {
                        "one_tp", null,
                        "Specifies whether the decoder should only decode the first "
                        + "tile part of each tile.",
                        "off"
                    },
                new string[]
                    {
                        "comp_transf", null,
                        "Specifies whether the component transform indicated in the "
                        + "codestream should be used.",
                        "on"
                    },
                new string[]
                    {
                        "debug", null,
                        "Print debugging messages when an error is encountered.",
                        "off"
                    },
                new string[]
                    {
                        "cdstr_info", null,
                        "Display information about the codestream. This information is: "
                        + "\n- Marker segments value in main and tile-part headers,"
                        + "\n- Tile-part length and position within the code-stream.",
                        "off"
                    },
                new string[]
                    {
                        "nocolorspace", null,
                        "Ignore any colorspace information in the image.", "off"
                    },
                new string[]
                    {
                        "colorspace_debug", null,
                        "Print debugging messages when an error is encountered in the"
                        + " colorspace module.",
                        "off"
                    }
            };

        #endregion

        #region Encoder Parameters

        private static String[][] encoder_pinfo =
            {
                new string[]
                    {
                        "debug", null,
                        "Print debugging messages when an error is encountered.",
                        "off"
                    },
                new string[]
                    {
                        "disable_jp2_extension", "[on|off]",
                        "JJ2000 automatically adds .jp2 extension when using 'file_format'"
                        + "option. This option disables it when on.",
                        "off"
                    },
                new string[]
                    {
                        "file_format", "[on|off]",
                        "Puts the JPEG 2000 codestream in a JP2 file format wrapper.",
                        "on"
                    },
                new string[]
                    {
                        "pph_tile", "[on|off]",
                        "Packs the packet headers in the tile headers.", "off"
                    },
                new string[]
                    {
                        "pph_main", "[on|off]",
                        "Packs the packet headers in the main header.", "off"
                    },
                new string[]
                    {
                        "pfile", "<filename of arguments file>",
                        "Loads the arguments from the specified file. Arguments that are "
                        + "specified on the command line override the ones from the file.\n"
                        + "The arguments file is a simple text file with one argument per "
                        + "line of the following form:\n"
                        + "  <argument name>=<argument value>\n"
                        + "If the argument is of boolean type (i.e. its presence turns a "
                        + "feature on), then the 'on' value turns it on, while the 'off' "
                        + "value turns it off. The argument name does not include the '-' "
                        + "or '+' character. Long lines can be broken into several lines "
                        + "by terminating them with '\'. Lines starting with '#' are "
                        + "considered as comments. This option is not recursive: any 'pfile' "
                        + "argument appearing in the file is ignored.",
                        null
                    },
                new string[]
                    {
                        "tile_parts", "<packets per tile-part>",
                        "This option specifies the maximum number of packets to have in "
                        + "one tile-part. 0 means include all packets in first tile-part "
                        + "of each tile",
                        "0"
                    },
                new string[]
                    {
                        "tiles", "<nominal tile width> <nominal tile height>",
                        "This option specifies the maximum tile dimensions to use. "
                        + "If both dimensions are 0 then no tiling is used.",
                        "0 0"
                    },
                new string[]
                    {
                        "ref", "<x> <y>",
                        "Sets the origin of the image in the canvas system. It sets the "
                        + "coordinate of the top-left corner of the image reference grid, "
                        + "with respect to the canvas origin",
                        "0 0"
                    },
                new string[]
                    {
                        "tref", "<x> <y>",
                        "Sets the origin of the tile partitioning on the reference grid, "
                        + "with respect to the canvas origin. The value of 'x' ('y') "
                        + "specified can not be larger than the 'x' one specified in the ref "
                        + "option.",
                        "0 0"
                    },
                new string[]
                    {
                        "rate", "<output bitrate in bpp>",
                        "This is the output bitrate of the codestream in bits per pixel."
                        + " When equal to -1, no image information (beside quantization "
                        + "effects) is discarded during compression.\n"
                        + "Note: In the case where '-file_format' option is used, the "
                        + "resulting file may have a larger bitrate.",
                        "-1"
                    },
                new string[]
                    {
                        "lossless", "[on|off]",
                        "Specifies a lossless compression for the encoder. This options"
                        + " is equivalent to use reversible quantization ('-Qtype "
                        + "reversible')"
                        + " and 5x3 wavelet filters pair ('-Ffilters w5x3'). Note that "
                        + "this option cannot be used with '-rate'. When this option is "
                        + "off, the quantization type and the filters pair is defined by "
                        + "'-Qtype' and '-Ffilters' respectively.",
                        "off"
                    },
                new string[]
                    {
                        "i", "<image file> [,<image file> [,<image file> ... ]]",
                        "Mandatory argument. This option specifies the name of the input "
                        + "image files. If several image files are provided, they have to be"
                        + " separated by commas in the command line. Supported formats are "
                        + "PGM (raw), PPM (raw) and PGX, "
                        + "which is a simple extension of the PGM file format for single "
                        + "component data supporting arbitrary bitdepths. If the extension "
                        + "is '.pgm', PGM-raw file format is assumed, if the extension is "
                        + "'.ppm', PPM-raw file format is assumed, otherwise PGX file "
                        + "format is assumed. PGM and PPM files are assumed to be 8 bits "
                        + "deep. A multi-component image can be specified by either "
                        + "specifying several PPM and/or PGX files, or by specifying one "
                        + "PPM file.",
                        null
                    },
                new string[]
                    {
                        "o", "<file name>",
                        "Mandatory argument. This option specifies the name of the output "
                        + "file to which the codestream will be written.",
                        null
                    },
                new string[]
                    {
                        "verbose", null,
                        "Prints information about the obtained bit stream.", "on"
                    },
                new string[]
                    {
                        "v", "[on|off]", "Prints version and copyright information.",
                        "off"
                    },
                new string[]
                    {
                        "u", "[on|off]",
                        "Prints usage information. "
                        + "If specified all other arguments (except 'v') are ignored",
                        "off"
                    },
            };

        #endregion

        /**
     * Prints the error message 'msg' to standard err, prepending "ERROR" to
     * it, and sets the exitCode to 'code'. An exit code different than 0
     * indicates that there where problems.
     *
     * @param msg The error message
     *
     * @param code The exit code to set
     * */

        [MethodImpl(MethodImplOptions.AggressiveInlining)]

        private static void error(String msg, int code)
        {
            //exitCode = code;
            FacilityManager.getMsgLogger().printmsg(MsgLogger_Fields.ERROR, msg);
        }

        /**
         * Prints the warning message 'msg' to standard err, prepending "WARNING"
         * to it.
         *
         * @param msg The error message
         * */

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void warning(String msg)
        {
            FacilityManager.getMsgLogger().printmsg(MsgLogger_Fields.WARNING, msg);
        }

        private static string GetImageType(Stream inStream)
        {
            try
            {
                var bytes = new byte[2];
                inStream.Position = 0;
                inStream.Read(bytes, 0, 2);
                inStream.Position = 0;
                var imgType = Encoding.UTF8.GetString(bytes, 0, 2);
                return imgType;
            }
            catch
            {
                return null;
            }
        }
    }
}
