/*
 * Copyright (c) 2006-2016, openmetaverse.co
 * All rights reserved.
 *
 * - Redistribution and use in source and binary forms, with or without 
 *   modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Neither the name of the openmetaverse.co nor the names 
 *   of its contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace OpenMetaverse
{
    /// <summary>
    /// Wrapper around a byte array that allows bit to be packed and unpacked
    /// one at a time or by a variable amount. Useful for very tightly packed
    /// data like LayerData packets
    /// </summary>
    public class BitPack
    {
        private const int MAX_BITS = 8;

        /// <summary></summary>
        public byte[] Data;

        /// <summary></summary>
        private int bytePos;
        public int BytePos
        {
            get
            {
                if (bytePos != 0 && bitPos == 0)
                    return bytePos - 1;
                else
                    return bytePos;
            }
        }

        /// <summary></summary>
        private int bitPos;
        public int BitPos { get { return bitPos; } }

        /// <summary>
        /// Default constructor, initialize the bit packer / bit unpacker
        /// with a byte array and starting position
        /// </summary>
        /// <param name="data">Byte array to pack bits in to or unpack from</param>
        /// <param name="pos">Starting position in the byte array</param>
        /// <param name="bitp">Optional bit position to start apendig more bits</param>
        public BitPack(byte[] data, int pos, int? bitp = null)
        {
            Data = data;
            bytePos = pos;
            if (bitp.HasValue)
            {
                bitPos = bitp.Value;
                if (bitPos < 0)
                    bitPos = 0;
                else if (bitPos > 7)
                    bitPos = 7; // this is wrong anyway
                if (bitPos == 0)
                    Data[pos] = 0;
                else
                    Data[pos] &= (byte)~(0xff >> bitPos);
            }
            else
                bitPos = 0;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private byte READBYTE()
        {
            return Unsafe.ReadUnaligned<byte>(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(Data), bytePos));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void BYTESTORE(byte b)
        {
            Unsafe.WriteUnaligned(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(Data), bytePos), b);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void OR_BYTESTORE(byte b)
        {
            byte o = Unsafe.ReadUnaligned<byte>(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(Data), bytePos));
            Unsafe.WriteUnaligned(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(Data), bytePos), b | o);
        }

        public void Reset(byte[] data, int pos, int? bitp = null)
        {
            Data = data;
            bytePos = pos;
            if (bitp.HasValue)
            {
                bitPos = bitp.Value;
                if (bitPos < 0)
                    bitPos = 0;
                else if (bitPos > 7)
                    bitPos = 7; // this is wrong anyway
                if (bitPos == 0)
                    Data[pos] = 0;
                else
                    Data[pos] &= (byte)~(0xff >> bitPos);
            }
            else
                bitPos = 0;
        }
        /// <summary>
        /// Pack a floating point value in to the data
        /// </summary>
        /// <param name="data">Floating point value to pack</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public unsafe void PackFloat(float data)
        {
            uint d = Unsafe.As<float, uint>(ref data);
            PackBitsFromByte((byte)d);
            PackBitsFromByte((byte)(d >> 8));
            PackBitsFromByte((byte)(d >> 16));
            PackBitsFromByte((byte)(d >> 24));
        }

        /// <summary>
        /// Pack part or all of an integer in to the data
        /// </summary>
        /// <param name="data">Integer containing the data to pack</param>
        /// <param name="totalCount">Number of bits of the integer to pack</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBits(int data, int totalCount)
        {
            PackBits((uint)data, totalCount);
        }

        /// <summary>
        /// Pack part or all of an unsigned integer in to the data
        /// </summary>
        /// <param name="data">Unsigned integer containing the data to pack</param>
        /// <param name="totalCount">Number of bits of the integer to pack</param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBits(uint data, int totalCount)
        {
            while (totalCount > 8)
            {
                PackBitsFromByte((byte)data);
                data >>= 8;
                totalCount -= 8;
            }
            if (totalCount > 0)
                PackBitsFromByte((byte)data, totalCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBitsFromUInt(uint data)
        {
            PackBitsFromByte((byte)data);
            PackBitsFromByte((byte)(data >> 8));
            PackBitsFromByte((byte)(data >> 16));
            PackBitsFromByte((byte)(data >> 24));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBitsFromInt(int data)
        {
            PackBitsFromUInt((uint)data);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBitsFromUShort(ushort data)
        {
            PackBitsFromByte((byte)data);
            PackBitsFromByte((byte)(data >> 8));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBitsFromShort(short data)
        {
            PackBitsFromUShort((ushort) data);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="data"></param>
        /// <param name="isSigned"></param>
        /// <param name="intBits"></param>
        /// <param name="fracBits"></param>
        public void PackFixed(float data, bool isSigned, int intBits, int fracBits)
        {
            int totalBits = intBits + fracBits;
            int max = 1 << intBits;

            if (isSigned)
            {
                totalBits++;
                data += max;
                max += max;
            }

            if (totalBits > 32)
                throw new Exception("Can't use fixed point packing for " + totalBits);

            int v;
            if(data <= 1e-6f)
                v = 0;
            else
            {
                if(data > max)
                    data = max;
                data *= 1 << fracBits;
                v = (int)data;
            }

            PackBitsFromByte((byte)v);
            if(totalBits > 8)
            {
                PackBitsFromByte((byte)(v >> 8));
                if (totalBits > 16)
                {
                    PackBitsFromByte((byte)(v >> 16));
                    PackBitsFromByte((byte)(v >> 24));
                }
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="data"></param>
        public void PackUUID(UUID data)
        {
            if (BitConverter.IsLittleEndian)
            {
                PackBitsFromByte(data.bytea3);
                PackBitsFromByte(data.bytea2);
                PackBitsFromByte(data.bytea1);
                PackBitsFromByte(data.bytea0);

                PackBitsFromByte(data.byteb1);
                PackBitsFromByte(data.byteb0);

                PackBitsFromByte(data.bytec1);
                PackBitsFromByte(data.bytec0);

                PackBitsFromByte(data.d);
                PackBitsFromByte(data.e);
                PackBitsFromByte(data.f);
                PackBitsFromByte(data.g);
                PackBitsFromByte(data.h);
                PackBitsFromByte(data.i);
                PackBitsFromByte(data.j);
                PackBitsFromByte(data.k);
            }
            else
            {
                PackBitsFromByte(data.bytea0);
                PackBitsFromByte(data.bytea1);
                PackBitsFromByte(data.bytea2);
                PackBitsFromByte(data.bytea3);

                PackBitsFromByte(data.byteb0);
                PackBitsFromByte(data.byteb1);

                PackBitsFromByte(data.bytec0);
                PackBitsFromByte(data.bytec1);

                PackBitsFromByte(data.k);
                PackBitsFromByte(data.j);
                PackBitsFromByte(data.i);
                PackBitsFromByte(data.h);
                PackBitsFromByte(data.g);
                PackBitsFromByte(data.f);
                PackBitsFromByte(data.e);
                PackBitsFromByte(data.d);
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="data"></param>
        public void PackColor(Color4 data)
        {
            PackBitsFromByte(Utils.FloatZeroOneToByte(data.R));
            PackBitsFromByte(Utils.FloatZeroOneToByte(data.G));
            PackBitsFromByte(Utils.FloatZeroOneToByte(data.B));
            PackBitsFromByte(Utils.FloatZeroOneToByte(data.A));
        }

        /// <summary>
        /// Unpacking a floating point value from the data
        /// </summary>
        /// <returns>Unpacked floating point value</returns>
        public float UnpackFloat()
        {
            byte[] output = UnpackBitsArray(32);

            if (!BitConverter.IsLittleEndian) Array.Reverse(output);
            return BitConverter.ToSingle(output, 0);
        }

        /// <summary>
        /// Unpack a variable number of bits from the data in to integer format
        /// </summary>
        /// <param name="totalCount">Number of bits to unpack</param>
        /// <returns>An integer containing the unpacked bits</returns>
        /// <remarks>This function is only useful up to 32 bits</remarks>
        public int UnpackBits(int totalCount)
        {
            byte[] output = UnpackBitsArray(totalCount);

            if (!BitConverter.IsLittleEndian) Array.Reverse(output);
            return BitConverter.ToInt32(output, 0);
        }

        /// <summary>
        /// Unpack a variable number of bits from the data in to unsigned 
        /// integer format
        /// </summary>
        /// <param name="totalCount">Number of bits to unpack</param>
        /// <returns>An unsigned integer containing the unpacked bits</returns>
        /// <remarks>This function is only useful up to 32 bits</remarks>
        public uint UnpackUBits(int totalCount)
        {
            byte[] output = UnpackBitsArray(totalCount);

            if (!BitConverter.IsLittleEndian) Array.Reverse(output);
            return BitConverter.ToUInt32(output, 0);
        }

        /// <summary>
        /// Unpack a 16-bit signed integer
        /// </summary>
        /// <returns>16-bit signed integer</returns>
        public short UnpackShort()
        {
            return (short)UnpackBits(16);
        }

        /// <summary>
        /// Unpack a 16-bit unsigned integer
        /// </summary>
        /// <returns>16-bit unsigned integer</returns>
        public ushort UnpackUShort()
        {
            return (ushort)UnpackUBits(16);
        }

        /// <summary>
        /// Unpack a 32-bit signed integer
        /// </summary>
        /// <returns>32-bit signed integer</returns>
        public int UnpackInt()
        {
            return (int)UnpackUInt();
        }

        /// <summary>
        /// Unpack a 32-bit unsigned integer
        /// </summary>
        /// <returns>32-bit unsigned integer</returns>
        public uint UnpackUInt()
        {
            uint tmp = UnpackByte();
            tmp |= (byte)(UnpackByte() << 8);
            tmp |= (byte)(UnpackByte() << 16);
            tmp |= (byte)(UnpackByte() << 24);
            return tmp;
        }
        public byte UnpackByte()
        {
            byte o = Data[bytePos];
            if (bitPos == 0 || o == 0)
            {
                ++bytePos;
                return o;
            }

            o <<= bitPos;
            ++bytePos;
            o |= (byte)(Data[bytePos] >> (8 - bitPos));
            return o;
        }

        public float UnpackFixed(bool signed, int intBits, int fracBits)
        {
            int totalBits = intBits + fracBits;
            if (signed)
                totalBits++;

            int intVal = UnpackByte();
            if (totalBits > 8)
            {
                intVal |= (UnpackByte() << 8);
                if (totalBits > 16)
                {
                    intVal |= (UnpackByte() << 16);
                    intVal |= (UnpackByte() << 24);
                }
            }

            if(intVal == 0)
                return 0f;

            float fixedVal = intVal;
            fixedVal /= (1 << fracBits);

            if (signed) fixedVal -= (1 << intBits);

            return fixedVal;
        }

        public string UnpackString(int size)
        {
            if (bitPos != 0 || bytePos + size > Data.Length) throw new IndexOutOfRangeException();

            string str = System.Text.UTF8Encoding.UTF8.GetString(Data, bytePos, size);
            bytePos += size;
            return str;
        }
        public UUID UnpackUUID()
        {
            if (bitPos != 0) throw new IndexOutOfRangeException();

            UUID val = new UUID(Data, bytePos);
            bytePos += 16;
            return val;
        }

        private void PackBitArray(byte[] data, int totalCount)
        {
            int count = 0;
            while(totalCount > 8)
            {
                PackBitsFromByte(data[count]);
                ++count;
                totalCount -= 8;
            }
            if(totalCount > 0)
                PackBitsFromByte(data[count], totalCount);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBitsFromByte(byte inbyte)
        {
            if(bitPos == 0)
            {
                BYTESTORE(inbyte);
                ++bytePos;
                if (bytePos < Data.Length)
                    BYTESTORE(0);
                return;
            }
            if (inbyte == 0)
            {
                ++bytePos;
                BYTESTORE(0);
                return;
            }
            OR_BYTESTORE((byte)(inbyte >> bitPos));
            ++bytePos;
            BYTESTORE((byte)(inbyte << (8 - bitPos)));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBitsFromByte(byte inbyte, int count)
        {
            if (count < 1)
                return;

            if (count > 8) //should not happen
                count = 7;
            else
                --count;

            byte cur = READBYTE();
            while (count > 0)
            {
                if ((inbyte & (0x01 << count)) != 0)
                    cur |= (byte)(0x80 >> bitPos);

                --count;
                ++bitPos;

                if (bitPos >= MAX_BITS)
                {
                    BYTESTORE(cur);
                    ++bytePos;
                    cur = 0;
                    bitPos = 0;
                }
            }

            if ((inbyte & 0x01 ) != 0)
                cur |= (byte)(0x80 >> bitPos);

            BYTESTORE(cur);
            ++bitPos;

            if (bitPos >= MAX_BITS)
            {
                bitPos = 0;
                ++bytePos;
                if (bytePos < Data.Length)
                    BYTESTORE(0);
            }
        }

        /// <summary>
        /// Pack a single bit in to the data
        /// </summary>
        /// <param name="bit">Bit to pack</param>

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void PackBit(bool bit)
        {
            if (bit)
                OR_BYTESTORE((byte)(0x80 >> bitPos));

            ++bitPos;

            if (bitPos >= MAX_BITS)
            {
                bitPos = 0;
                ++bytePos;
                if (bytePos < Data.Length)
                    BYTESTORE(0);
            }
        }

        private byte[] UnpackBitsArray(int totalCount)
        {
            int count = 0;
            byte[] output = new byte[4];
            int curBytePos = 0;
            int curBitPos = 0;

            while (totalCount > 0)
            {
                if (totalCount > MAX_BITS)
                {
                    count = MAX_BITS;
                    totalCount -= MAX_BITS;
                }
                else
                {
                    count = totalCount;
                    totalCount = 0;
                }

                while (count > 0)
                {
                    // Shift the previous bits
                    output[curBytePos] <<= 1;

                    // Grab one bit
                    if ((Data[bytePos] & (0x80 >> bitPos++)) != 0)
                        ++output[curBytePos];

                    --count;
                    ++curBitPos;

                    if (bitPos >= MAX_BITS)
                    {
                        bitPos = 0;
                        ++bytePos;
                    }
                    if (curBitPos >= MAX_BITS)
                    {
                        curBitPos = 0;
                        ++curBytePos;
                    }
                }
            }

            return output;
        }
    }
}
