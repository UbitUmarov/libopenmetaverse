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
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Text;
using System.Runtime.CompilerServices;

namespace OpenMetaverse.StructuredData
{
    /// <summary>
    /// 
    /// </summary>
    public enum OSDType:byte
    {
        /// <summary></summary>
        Unknown,
        /// <summary></summary>
        Boolean,
        /// <summary></summary>
        Integer,
        /// <summary></summary>
        Real,
        /// <summary></summary>
        String,
        /// <summary></summary>
        UUID,
        /// <summary></summary>
        Date,
        /// <summary></summary>
        URI,
        /// <summary></summary>
        Binary,
        /// <summary></summary>
        Map,
        /// <summary></summary>
        Array,
        LLSDxml,
        OSDUTF8
    }

    public enum OSDFormat
    {
        Xml = 0,
        Json,
        Binary
    }

    /// <summary>
    /// 
    /// </summary>
    public class OSDException : Exception
    {
        public OSDException(string message) : base(message) { }
    }

    /// <summary>
    /// 
    /// </summary>
    public partial class OSD
    {
        protected static readonly byte[] trueBinary = { 0x31 };
        protected static readonly byte[] falseBinary = { 0x30 };

        public OSDType Type = OSDType.Unknown;

        // .net4.8 64Bit JIT fails polimorphism
        public virtual bool AsBoolean()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value;
                case OSDType.Integer:
                    return ((OSDInteger)this).value != 0;
                case OSDType.Real:
                    double d = ((OSDReal)this).value;
                    return (!Double.IsNaN(d) && d != 0);
                case OSDType.String:
                    if (string.IsNullOrEmpty(((OSDString)this).value) ||
                            ((OSDString)this).value.Equals("0") ||
                            ((OSDString)this).value.Equals("false", StringComparison.InvariantCultureIgnoreCase))
                        return false;
                    return true;
                case OSDType.UUID:
                    return ((OSDUUID)this).value.IsNotZero();
                case OSDType.Map:
                    return ((OSDMap)this).dicvalue.Count > 0;
                case OSDType.Array:
                    return ((OSDArray)this).value.Count > 0;
                case OSDType.OSDUTF8:
                    osUTF8 u = ((OSDUTF8)this).value;
                    if (osUTF8.IsNullOrEmpty(u))
                        return false;
                    if (u.Equals('0') || u.ACSIILowerEquals("false"))
                        return false;
                    return true;

                default:
                    return false;
            }
        }

        public virtual int AsInteger()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value ? 1 : 0;
                case OSDType.Integer:
                    return ((OSDInteger)this).value;
                case OSDType.Real:
                    double v = ((OSDReal)this).value;
                    if (Double.IsNaN(v))
                        return 0;
                    if (v >= Int32.MaxValue)
                        return Int32.MaxValue;
                    if (v <= Int32.MinValue)
                        return Int32.MinValue;
                    return (int)Math.Round(v);
                case OSDType.String:
                    if (Double.TryParse(((OSDString)this).value.AsSpan(), out double dbl))
                        return (int)Math.Floor(dbl);
                    else
                        return 0;
                case OSDType.OSDUTF8:
                    var us = ((OSDUTF8)this).value.ToString().AsSpan();
                    if (Double.TryParse(us, out double udbl))
                        return (int)Math.Floor(udbl);
                    else
                        return 0;
                case OSDType.Binary:
                    byte[] b = ((OSDBinary)this).value;
                    if (b.Length < 4)
                        return 0;
                    return (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    if (l.Count < 4)
                        return 0;
                    return
                        (byte)l[0].AsInteger() << 24 |
                        (byte)l[1].AsInteger() << 16 |
                        (byte)l[2].AsInteger() << 8 |
                        (byte)l[3].AsInteger();

                case OSDType.Date:
                    return (int)Utils.DateTimeToUnixTime(((OSDDate)this).value);
                default:
                    return 0;
            }
        }

        public virtual uint AsUInteger()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value ? 1U : 0;
                case OSDType.Integer:
                    return (uint)((OSDInteger)this).value;
                case OSDType.Real:
                    double v = ((OSDReal)this).value;
                    if (Double.IsNaN(v))
                        return 0;
                    if (v > UInt32.MaxValue)
                        return UInt32.MaxValue;
                    if (v < UInt32.MinValue)
                        return UInt32.MinValue;
                    return (uint)Math.Round(v);
                case OSDType.String:
                    if (Double.TryParse(((OSDString)this).value.AsSpan(), out double dbl))
                        return (uint)Math.Floor(dbl);
                    else
                        return 0;
                case OSDType.OSDUTF8:
                    if (Double.TryParse(((OSDUTF8)this).value.ToString().AsSpan(), out double udbl))
                        return (uint)Math.Floor(udbl);
                    else
                        return 0;
                case OSDType.Date:
                    return Utils.DateTimeToUnixTime(((OSDDate)this).value);
                case OSDType.Binary:
                    byte[] b = ((OSDBinary)this).value;
                    if(b.Length < 4)
                        return 0;
                    return (uint)(
                        (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3]);
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    if (l.Count < 4)
                        return 0;
                    return (
                        ((uint)(byte)l[0].AsInteger() << 24) |
                        ((uint)(byte)l[1].AsInteger() << 16) |
                        ((uint)(byte)l[2].AsInteger() << 8) |
                        (byte)l[3].AsInteger());
                default:
                    return 0;
            }
        }

        public virtual long AsLong()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value ? 1 : 0;
                case OSDType.Integer:
                    return ((OSDInteger)this).value;
                case OSDType.Real:
                    double v = ((OSDReal)this).value;
                    if (Double.IsNaN(v))
                        return 0;
                    if (v > Int64.MaxValue)
                        return Int64.MaxValue;
                    if (v < Int64.MinValue)
                        return Int64.MinValue;
                    return (long)Math.Round(v);
                case OSDType.String:
                    if (Double.TryParse(((OSDString)this).value, out double dbl))
                        return (long)Math.Floor(dbl);
                    else
                        return 0;
                case OSDType.OSDUTF8:
                    var us = ((OSDUTF8)this).value.ToString().AsSpan();
                    if (Double.TryParse(us, out double udbl))
                        return (long)Math.Floor(udbl);
                    else
                        return 0;
                case OSDType.Date:
                    return Utils.DateTimeToUnixTime(((OSDDate)this).value);
                case OSDType.Binary:
                {
                    byte[] b = ((OSDBinary)this).value;
                    if(b.Length < 8)
                        return 0;
                    return (
                        ((long)b[0] << 56) |
                        ((long)b[1] << 48) |
                        ((long)b[2] << 40) |
                        ((long)b[3] << 32) |
                        ((long)b[4] << 24) |
                        ((long)b[5] << 16) |
                        ((long)b[6] << 8) |
                        b[7]);
                }
                case OSDType.Array:
                {
                    List<OSD> l = ((OSDArray)this).value;
                    if (l.Count < 8)
                        return 0;
                    return 
                        ((long)(byte)l[0].AsInteger() << 56) |
                        ((long)(byte)l[1].AsInteger() << 48) |
                        ((long)(byte)l[2].AsInteger() << 40) |
                        ((long)(byte)l[3].AsInteger() << 32) |
                        ((long)(byte)l[4].AsInteger() << 24) |
                        ((long)(byte)l[5].AsInteger() << 16) |
                        ((long)(byte)l[6].AsInteger() << 8) |
                        (byte)l[7].AsInteger();
                }
                default:
                    return 0;
            }
        }

        public virtual ulong AsULong()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value ? 1UL : 0;
                case OSDType.Integer:
                    return (ulong)((OSDInteger)this).value;
                case OSDType.Real:
                    double v = ((OSDReal)this).value;
                    if (Double.IsNaN(v))
                        return 0;
                    if (v > UInt64.MaxValue)
                        return UInt64.MaxValue;
                    if (v < UInt64.MinValue)
                        return UInt64.MinValue;
                    return (ulong)Math.Round(v);
                case OSDType.String:
                    if (Double.TryParse(((OSDString)this).value.AsSpan(), out double dbl))
                        return (ulong)Math.Floor(dbl);
                    else
                        return 0;
                case OSDType.OSDUTF8:
                    if (Double.TryParse(((OSDUTF8)this).value.ToString().AsSpan(), out double udbl))
                        return (ulong)Math.Floor(udbl);
                    else
                        return 0;
                case OSDType.Date:
                    return Utils.DateTimeToUnixTime(((OSDDate)this).value);
                case OSDType.Binary:
                {
                    byte[] b = ((OSDBinary)this).value;
                    if (b.Length < 8)
                        return 0;
                    return (
                        ((ulong)b[0] << 56) |
                        ((ulong)b[1] << 48) |
                        ((ulong)b[2] << 40) |
                        ((ulong)b[3] << 32) |
                        ((ulong)b[4] << 24) |
                        ((ulong)b[5] << 16) |
                        ((ulong)b[6] << 8) |
                        b[7]);
                }
                case OSDType.Array:
                {
                    List<OSD> l = ((OSDArray)this).value;
                    if (l.Count < 8)
                        return 0;
                    return (
                        ((ulong)(byte)l[0].AsInteger() << 56) |
                        ((ulong)(byte)l[1].AsInteger() << 48) |
                        ((ulong)(byte)l[2].AsInteger() << 40) |
                        ((ulong)(byte)l[3].AsInteger() << 32) |
                        ((ulong)(byte)l[4].AsInteger() << 24) |
                        ((ulong)(byte)l[5].AsInteger() << 16) |
                        ((ulong)(byte)l[6].AsInteger() << 8) |
                        (byte)l[7].AsInteger());
                }
                default:
                    return 0;
            }
        }

        public virtual double AsReal()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value ? 1.0 : 0;
                case OSDType.Integer:
                    return ((OSDInteger)this).value;
                case OSDType.Real:
                    return ((OSDReal)this).value;
                case OSDType.String:
                    if (Double.TryParse(((OSDString)this).value.AsSpan(), out double dbl))
                        return dbl;
                    else
                        return 0;
                case OSDType.OSDUTF8:
                    if (Double.TryParse(((OSDUTF8)this).value.ToString().AsSpan(), out double udbl))
                        return udbl;
                    else
                        return 0;
                default:
                    return 0;
            }
        }

        public virtual string AsString()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value ? "1" : "0";
                case OSDType.Integer:
                    return ((OSDInteger)this).value.ToString();
                case OSDType.Real:
                    return ((OSDReal)this).value.ToString("g", Utils.EnUsCulture);
                case OSDType.String:
                    return ((OSDString)this).value;
                case OSDType.OSDUTF8:
                    return ((OSDUTF8)this).value.ToString();
                case OSDType.UUID:
                    return ((OSDUUID)this).value.ToString();
                case OSDType.Date:
                    string format;
                    DateTime dt = ((OSDDate)this).value;
                    if (dt.Millisecond > 0)
                        format = "yyyy-MM-ddTHH:mm:ss.ffZ";
                    else
                        format = "yyyy-MM-ddTHH:mm:ssZ";
                    return dt.ToUniversalTime().ToString(format);
                case OSDType.URI:
                    Uri ur = ((OSDUri)this).value;
                    if (ur == null)
                        return string.Empty;
                    if (ur.IsAbsoluteUri)
                        return ur.AbsoluteUri;
                    else
                        return ur.ToString();

                case OSDType.Binary:
                    byte[] b = ((OSDBinary)this).value;
                    return Convert.ToBase64String(b);
                case OSDType.LLSDxml:
                    return ((OSDllsdxml)this).value;
                default:
                    return String.Empty;
            }
        }

        public virtual UUID AsUUID()
        {
            switch (Type)
            {
                case OSDType.String:
                    if (UUID.TryParse(((OSDString)this).value.AsSpan(), out UUID uuid))
                        return uuid;
                    else
                        return UUID.Zero;
                case OSDType.OSDUTF8:
                    UUID ouuid;
                    if (UUID.TryParse(((OSDUTF8)this).value.ToString().AsSpan(), out ouuid))
                        return ouuid;
                    else
                        return UUID.Zero;
                case OSDType.UUID:
                    return ((OSDUUID)this).value;
                default:
                    return UUID.Zero;
            }
        }

        public virtual DateTime AsDate()
        {
            switch (Type)
            {
                case OSDType.String:
                    DateTime dt;
                    if (DateTime.TryParse(((OSDString)this).value, out dt))
                        return dt;
                    else
                        return Utils.Epoch;
                case OSDType.OSDUTF8:
                    DateTime odt;
                    if (DateTime.TryParse(((OSDUTF8)this).value.ToString(), out odt))
                        return odt;
                    else
                        return Utils.Epoch;
                case OSDType.UUID:
                case OSDType.Date:
                    return ((OSDDate)this).value;
                default:
                    return Utils.Epoch;
            }
        }
        public virtual Uri AsUri()
        {
            switch (Type)
            {
                case OSDType.String:
                    Uri uri;
                    if (Uri.TryCreate(((OSDString)this).value, UriKind.RelativeOrAbsolute, out uri))
                        return uri;
                    else
                        return null;
                case OSDType.OSDUTF8:
                    Uri ouri;
                    if (Uri.TryCreate(((OSDUTF8)this).value.ToString(), UriKind.RelativeOrAbsolute, out ouri))
                        return ouri;
                    else
                        return null;
                case OSDType.URI:
                    return ((OSDUri)this).value;
                default:
                    return null;
            }
        }

        public virtual byte[] AsBinary()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value ? trueBinary : falseBinary;
                case OSDType.Integer:
                    return Utils.IntToBytesBig(((OSDInteger)this).value);
                case OSDType.Real:
                    return Utils.DoubleToBytesBig(((OSDReal)this).value);
                case OSDType.String:
                    return Encoding.UTF8.GetBytes(((OSDString)this).value);
                case OSDType.OSDUTF8:
                    return ((OSDUTF8)this).value.ToArray();
                case OSDType.UUID:
                    return (((OSDUUID)this).value).GetBytes();
                case OSDType.Date:
                    TimeSpan ts = (((OSDDate)this).value).ToUniversalTime() - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
                    return Utils.DoubleToBytes(ts.TotalSeconds);
                case OSDType.URI:
                    return Encoding.UTF8.GetBytes(((OSDUri)this).AsString());
                case OSDType.Binary:
                    return ((OSDBinary)this).value;
                case OSDType.Map:
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    byte[] binary = new byte[l.Count];
                    for (int i = 0; i < l.Count; i++)
                        binary[i] = (byte)l[i].AsInteger();
                    return binary;
                case OSDType.LLSDxml:
                    return Encoding.UTF8.GetBytes(((OSDllsdxml)this).value);
                default:
                    return Array.Empty<byte>();
            }
        }

        public Vector2 AsVector2()
        {
            switch (Type)
            {
                case OSDType.String:
                    return Vector2.Parse(((OSDString)this).value);
                case OSDType.OSDUTF8:
                    return Vector2.Parse(((OSDUTF8)this).value.ToString());
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    Vector2 vector = Vector2.Zero;
                    if (l.Count == 2)
                    {
                        vector.X = (float)l[0].AsReal();
                        vector.Y = (float)l[1].AsReal();
                    }
                    return vector;
                default:
                    return Vector2.Zero;
            }
        }

        public Vector3 AsVector3()
        {
            switch (Type)
            {
                case OSDType.String:
                    return Vector3.Parse(((OSDString)this).value.AsSpan());
                case OSDType.OSDUTF8:
                    return Vector3.Parse(((OSDUTF8)this).value.ToString().AsSpan());
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    if (l.Count == 3)
                    {
                        return new Vector3(
                            (float)l[0].AsReal(),
                            (float)l[1].AsReal(),
                            (float)l[2].AsReal());
                    }
                    return Vector3.Zero;
                default:
                    return Vector3.Zero;
            }
        }

        public Vector3d AsVector3d()
        {
            switch (Type)
            {
                case OSDType.String:
                    return Vector3d.Parse(((OSDString)this).value.AsSpan());
                case OSDType.OSDUTF8:
                    return Vector3d.Parse(((OSDUTF8)this).value.ToString().AsSpan());
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    Vector3d vector = Vector3d.Zero;
                    if (l.Count == 3)
                    {
                        vector.X = (float)l[0].AsReal();
                        vector.Y = (float)l[1].AsReal();
                        vector.Z = (float)l[2].AsReal();
                    }
                    return vector;
                default:
                    return Vector3d.Zero;
            }
        }

        public Vector4 AsVector4()
        {
            switch (Type)
            {
                case OSDType.String:
                    return Vector4.Parse(((OSDString)this).value);
                case OSDType.OSDUTF8:
                    return Vector4.Parse(((OSDUTF8)this).value.ToString());
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    Vector4 vector = Vector4.Zero;
                    if (l.Count == 4)
                    {
                        vector.X = (float)l[0].AsReal();
                        vector.Y = (float)l[1].AsReal();
                        vector.Z = (float)l[2].AsReal();
                        vector.W = (float)l[3].AsReal();
                    }
                    return vector;
                default:
                    return Vector4.Zero;
            }
        }

        public Quaternion AsQuaternion()
        {
            switch (Type)
            {
                case OSDType.String:
                    return Quaternion.Parse(((OSDString)this).value);
                case OSDType.OSDUTF8:
                    return Quaternion.Parse(((OSDString)this).value.ToString());
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    Quaternion q = Quaternion.Identity;
                    if (l.Count == 4)
                    {
                        q.X = (float)l[0].AsReal();
                        q.Y = (float)l[1].AsReal();
                        q.Z = (float)l[2].AsReal();
                        q.W = (float)l[3].AsReal();
                    }
                    return q;
                default:
                    return Quaternion.Identity;
            }
        }

        public virtual Color4 AsColor4()
        {
            switch (Type)
            {
                case OSDType.Array:
                    List<OSD> l = ((OSDArray)this).value;
                    Color4 color = Color4.Black;
                    if (l.Count == 4)
                    {
                        color.R = (float)l[0].AsReal();
                        color.G = (float)l[1].AsReal();
                        color.B = (float)l[2].AsReal();
                        color.A = (float)l[3].AsReal();
                    }
                    return color;
                default:
                    return Color4.Black;
            }
        }

        public virtual void Clear() { }

        public virtual OSD Copy()
        {
            return Type switch
            {
                OSDType.Boolean => new OSDBoolean(((OSDBoolean)this).value),
                OSDType.Integer => new OSDInteger(((OSDInteger)this).value),
                OSDType.Real => new OSDReal(((OSDReal)this).value),
                OSDType.String => new OSDString(((OSDString)this).value),
                OSDType.OSDUTF8 => new OSDUTF8(((OSDUTF8)this).value),
                OSDType.UUID => new OSDUUID(((OSDUUID)this).value),
                OSDType.Date => new OSDDate(((OSDDate)this).value),
                OSDType.URI => new OSDUri(((OSDUri)this).value),
                OSDType.Binary => new OSDBinary(((OSDBinary)this).value),
                OSDType.Map => new OSDMap(((OSDMap)this).dicvalue),
                OSDType.Array => new OSDArray(((OSDArray)this).value),
                OSDType.LLSDxml => new OSDBoolean(((OSDBoolean)this).value),
                _ => new OSD(),
            };
        }

        public override string ToString()
        {
            switch (Type)
            {
                case OSDType.Boolean:
                    return ((OSDBoolean)this).value ? "1" : "0";
                case OSDType.Integer:
                    return ((OSDInteger)this).value.ToString();
                case OSDType.Real:
                    return ((OSDReal)this).value.ToString("g", Utils.EnUsCulture);
                case OSDType.String:
                    return ((OSDString)this).value;
                case OSDType.OSDUTF8:
                    return ((OSDUTF8)this).value.ToString();
                case OSDType.UUID:
                    return ((OSDUUID)this).value.ToString();
                case OSDType.Date:
                    string format;
                    DateTime dt = ((OSDDate)this).value;
                    if (dt.Millisecond > 0)
                        format = "yyyy-MM-ddTHH:mm:ss.ffZ";
                    else
                        format = "yyyy-MM-ddTHH:mm:ssZ";
                    return dt.ToUniversalTime().ToString(format);
                case OSDType.URI:
                    Uri ur = ((OSDUri)this).value;
                    if (ur == null)
                        return string.Empty;
                    if (ur.IsAbsoluteUri)
                        return ur.AbsoluteUri;
                    else
                        return ur.ToString();
                case OSDType.Binary:
                    return Utils.BytesToHexString(((OSDBinary)this).value, null);
                case OSDType.LLSDxml:
                    return ((OSDllsdxml)this).value;
                case OSDType.Map:
                    return OSDParser.SerializeJsonString((OSDMap)this, true);
                case OSDType.Array:
                    return OSDParser.SerializeJsonString((OSDArray)this, true);
                default:
                    return "undef";
            }
        }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromBoolean(bool value) { return new OSDBoolean(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromInteger(int value) { return new OSDInteger(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromInteger(uint value) { return new OSDInteger((int)value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromInteger(short value) { return new OSDInteger((int)value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromInteger(ushort value) { return new OSDInteger((int)value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromInteger(sbyte value) { return new OSDInteger((int)value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromInteger(byte value) { return new OSDInteger((int)value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static OSD FromUInteger(uint value) { return new OSDBinary(value); }
        public static OSD FromLong(long value) { return new OSDBinary(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static OSD FromULong(ulong value) { return new OSDBinary(value); }
        public static OSD FromReal(double value) { return new OSDReal(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)] public static OSD FromReal(float value) { return new OSDReal((double)value); }
        public static OSD FromString(string value) { return new OSDString(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromUUID(UUID value) { return new OSDUUID(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromDate(DateTime value) { return new OSDDate(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromUri(Uri value) { return new OSDUri(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)] 
        public static OSD FromBinary(byte[] value) { return new OSDBinary(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromVector2(Vector2 value)
        {
            return new OSDArray() { OSD.FromReal(value.X), OSD.FromReal(value.Y) };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromVector3(Vector3 value)
        {
            return new OSDArray() { OSD.FromReal(value.X), OSD.FromReal(value.Y), OSD.FromReal(value.Z) };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromVector3d(Vector3d value)
        {
            return new OSDArray() { OSD.FromReal(value.X), OSD.FromReal(value.Y), OSD.FromReal(value.Z) };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromVector4(Vector4 value)
        {
            return new OSDArray() { OSD.FromReal(value.X), OSD.FromReal(value.Y), OSD.FromReal(value.Z), OSD.FromReal(value.W) };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromQuaternion(Quaternion value)
        {
            return new OSDArray() { OSD.FromReal(value.X), OSD.FromReal(value.Y), OSD.FromReal(value.Z), OSD.FromReal(value.W) };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static OSD FromColor4(Color4 value)
        {
            return new OSDArray() { OSD.FromReal(value.R), OSD.FromReal(value.G), OSD.FromReal(value.B), OSD.FromReal(value.A) };
        }

        public static OSD FromObject(object value)
        {
            if (value == null) { return new OSD(); }
            else if (value is bool bv) { return new OSDBoolean(bv); }
            else if (value is int iv) { return new OSDInteger(iv); }
            else if (value is uint uiv) { return new OSDBinary(uiv); }
            else if (value is short sv) { return new OSDInteger((int)sv); }
            else if (value is ushort usv) { return new OSDInteger((int)usv); }
            else if (value is sbyte sbv) { return new OSDInteger((int)sbv); }
            else if (value is byte btv) { return new OSDInteger((int)btv); }
            else if (value is double dv) { return new OSDReal(dv); }
            else if (value is float fv) { return new OSDReal((double)fv); }
            else if (value is string stv) { return new OSDString(stv); }
            else if (value is UUID uidv) { return new OSDUUID(uidv); }
            else if (value is DateTime dtmv) { return new OSDDate(dtmv); }
            else if (value is Uri uriv) { return new OSDUri(uriv); }
            else if (value is byte[] btav) { return new OSDBinary(btav); }
            else if (value is long lv) { return new OSDBinary(lv); }
            else if (value is ulong ulv) { return new OSDBinary(ulv); }
            else if (value is Vector2 v2v) { return FromVector2(v2v); }
            else if (value is Vector3 v3v) { return FromVector3(v3v); }
            else if (value is Vector3d v3dv) { return FromVector3d(v3dv); }
            else if (value is Vector4 v4v) { return FromVector4(v4v); }
            else if (value is Quaternion qv) { return FromQuaternion(qv); }
            else if (value is Color4 c4v) { return FromColor4(c4v); }
            else return new OSD();
        }

        public static object ToObject(Type type, OSD value)
        {
            if (type == typeof(ulong))
            {
                if (value.Type == OSDType.Binary)
                {
                    byte[] bytes = value.AsBinary();
                    return Utils.BytesToUInt64(bytes);
                }
                else
                {
                    return (ulong)value.AsInteger();
                }
            }
            else if (type == typeof(uint))
            {
                if (value.Type == OSDType.Binary)
                {
                    byte[] bytes = value.AsBinary();
                    return Utils.BytesToUInt(bytes);
                }
                else
                {
                    return (uint)value.AsInteger();
                }
            }
            else if (type == typeof(ushort))
            {
                return (ushort)value.AsInteger();
            }
            else if (type == typeof(byte))
            {
                return (byte)value.AsInteger();
            }
            else if (type == typeof(short))
            {
                return (short)value.AsInteger();
            }
            else if (type == typeof(string))
            {
                return value.AsString();
            }
            else if (type == typeof(bool))
            {
                return value.AsBoolean();
            }
            else if (type == typeof(float))
            {
                return (float)value.AsReal();
            }
            else if (type == typeof(double))
            {
                return value.AsReal();
            }
            else if (type == typeof(int))
            {
                return value.AsInteger();
            }
            else if (type == typeof(UUID))
            {
                return value.AsUUID();
            }
            else if (type == typeof(Vector3))
            {
                if (value.Type == OSDType.Array)
                    return ((OSDArray)value).AsVector3();
                else
                    return Vector3.Zero;
            }
            else if (type == typeof(Vector4))
            {
                if (value.Type == OSDType.Array)
                    return ((OSDArray)value).AsVector4();
                else
                    return Vector4.Zero;
            }
            else if (type == typeof(Quaternion))
            {
                if (value.Type == OSDType.Array)
                    return ((OSDArray)value).AsQuaternion();
                else
                    return Quaternion.Identity;
            }
            else if (type == typeof(OSDArray))
            {
                OSDArray newArray = new();
                foreach (OSD o in (OSDArray)value)
                    newArray.Add(o);
                return newArray;
            }
            else if (type == typeof(OSDMap))
            {
                OSDMap newMap = new();
                foreach (KeyValuePair<string, OSD> o in (OSDMap)value)
                    newMap.Add(o);
                return newMap;
            }
            else
            {
                return null;
            }
        }

        #region Implicit Conversions

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(bool value) { return new OSDBoolean(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(int value) { return new OSDInteger(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(uint value) { return new OSDInteger((int)value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(short value) { return new OSDInteger((int)value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(ushort value) { return new OSDInteger((int)value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(sbyte value) { return new OSDInteger((int)value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(byte value) { return new OSDInteger((int)value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(long value) { return new OSDBinary(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(ulong value) { return new OSDBinary(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(double value) { return new OSDReal(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(float value) { return new OSDReal(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(string value) { return new OSDString(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(UUID value) { return new OSDUUID(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(DateTime value) { return new OSDDate(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(Uri value) { return new OSDUri(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(byte[] value) { return new OSDBinary(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(Vector2 value) { return OSD.FromVector2(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(Vector3 value) { return OSD.FromVector3(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(Vector3d value) { return OSD.FromVector3d(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(Vector4 value) { return OSD.FromVector4(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(Quaternion value) { return OSD.FromQuaternion(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator OSD(Color4 value) { return OSD.FromColor4(value); }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator bool(OSD value) { return value.AsBoolean(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator int(OSD value) { return value.AsInteger(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator uint(OSD value) { return value.AsUInteger(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator long(OSD value) { return value.AsLong(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator ulong(OSD value) { return value.AsULong(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator double(OSD value) { return value.AsReal(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator float(OSD value) { return (float)value.AsReal(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator string(OSD value) { return value.AsString(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator UUID(OSD value) { return value.AsUUID(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator DateTime(OSD value) { return value.AsDate(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Uri(OSD value) { return value.AsUri(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Vector2(OSD value) { return value.AsVector2(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator byte[](OSD value) { return value.AsBinary(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Vector3(OSD value) { return value.AsVector3(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Vector3d(OSD value) { return value.AsVector3d(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Vector4(OSD value) { return value.AsVector4(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Quaternion(OSD value) { return value.AsQuaternion(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static implicit operator Color4(OSD value) { return value.AsColor4(); }

        #endregion Implicit Conversions

        /// <summary>
        /// Uses reflection to create an SDMap from all of the SD
        /// serializable types in an object
        /// </summary>
        /// <param name="obj">Class or struct containing serializable types</param>
        /// <returns>An SDMap holding the serialized values from the
        /// container object</returns>
        public static OSDMap SerializeMembers(object obj)
        {
            Type t = obj.GetType();
            FieldInfo[] fields = t.GetFields();

            OSDMap map = new(fields.Length);

            for (int i = 0; i < fields.Length; i++)
            {
                FieldInfo field = fields[i];
                if (!Attribute.IsDefined(field, typeof(NonSerializedAttribute)))
                {
                    OSD serializedField = OSD.FromObject(field.GetValue(obj));

                    if (serializedField.Type != OSDType.Unknown || field.FieldType == typeof(string) || field.FieldType == typeof(byte[]))
                        map.Add(field.Name, serializedField);
                }
            }

            return map;
        }

        /// <summary>
        /// Uses reflection to deserialize member variables in an object from
        /// an SDMap
        /// </summary>
        /// <param name="obj">Reference to an object to fill with deserialized
        /// values</param>
        /// <param name="serialized">Serialized values to put in the target
        /// object</param>
        public static void DeserializeMembers(ref object obj, OSDMap serialized)
        {
            Type t = obj.GetType();
            FieldInfo[] fields = t.GetFields();

            for (int i = 0; i < fields.Length; i++)
            {
                FieldInfo field = fields[i];
                if (!Attribute.IsDefined(field, typeof(NonSerializedAttribute)))
                {
                    if (serialized.TryGetValue(field.Name, out OSD serializedField))
                        field.SetValue(obj, ToObject(field.FieldType, serializedField));
                }
            }
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDBoolean : OSD
    {
        public readonly bool value;

        public OSDBoolean(bool value)
        {
            Type = OSDType.Boolean;
            this.value = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool AsBoolean() { return value; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int AsInteger() { return value ? 1 : 0; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override double AsReal() { return value ? 1d : 0d; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string AsString() { return value ? "1" : "0"; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override byte[] AsBinary() { return value ? trueBinary : falseBinary; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override OSD Copy() { return new OSDBoolean(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() { return AsString(); }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDInteger : OSD
    {
        public readonly int value;

        public OSDInteger(int value)
        {
            Type = OSDType.Integer;
            this.value = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool AsBoolean() { return value != 0; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override int AsInteger() { return value; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override uint AsUInteger() { return (uint)value; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override long AsLong() { return value; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override ulong AsULong() { return (ulong)value; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override double AsReal() { return (double)value; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string AsString() { return value.ToString(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override byte[] AsBinary() { return Utils.IntToBytesBig(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override OSD Copy() { return new OSDInteger(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() { return AsString(); }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDReal : OSD
    {
        public readonly double value;

        public OSDReal(double value)
        {
            Type = OSDType.Real;
            this.value = value;
        }

        public override bool AsBoolean() { return (!Double.IsNaN(value) && value != 0d); }
        public override OSD Copy() { return new OSDReal(value); }
        public override int AsInteger()
        {
            if (Double.IsNaN(value))
                return 0;
            if (value > (double)Int32.MaxValue)
                return Int32.MaxValue;
            if (value < (double)Int32.MinValue)
                return Int32.MinValue;
            return (int)Math.Round(value);
        }

        public override uint AsUInteger()
        {
            if (Double.IsNaN(value))
                return 0;
            if (value > (double)UInt32.MaxValue)
                return UInt32.MaxValue;
            if (value < (double)UInt32.MinValue)
                return UInt32.MinValue;
            return (uint)Math.Round(value);
        }

        public override long AsLong()
        {
            if (Double.IsNaN(value))
                return 0;
            if (value > (double)Int64.MaxValue)
                return Int64.MaxValue;
            if (value < (double)Int64.MinValue)
                return Int64.MinValue;
            return (long)Math.Round(value);
        }

        public override ulong AsULong()
        {
            if (Double.IsNaN(value))
                return 0;
            if (value > (double)UInt64.MaxValue)
                return Int32.MaxValue;
            if (value < (double)UInt64.MinValue)
                return UInt64.MinValue;
            return (ulong)Math.Round(value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override double AsReal() { return value; }
        // "r" ensures the value will correctly round-trip back through Double.TryParse
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string AsString() { return value.ToString("g", Utils.EnUsCulture); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override byte[] AsBinary() { return Utils.DoubleToBytesBig(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() { return AsString(); }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDllsdxml : OSD
    {
        public readonly string value;

        public override OSD Copy() { return new OSDllsdxml(value); }

        public OSDllsdxml(string value)
        {
            Type = OSDType.LLSDxml;
            // Refuse to hold null pointers
            if (value != null)
                this.value = value;
            else
                this.value = String.Empty;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string AsString() { return value; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override byte[] AsBinary() { return Encoding.UTF8.GetBytes(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() { return AsString(); }
    }

    public sealed class OSDUTF8 : OSD
    {
        public readonly osUTF8 value;

        public override OSD Copy() { return new OSDUTF8(value.Clone()); }

        public OSDUTF8(osUTF8 value)
        {
            Type = OSDType.OSDUTF8;
            // Refuse to hold null pointers
            if (value != null)
                this.value = value;
            else
                this.value = new osUTF8();
        }

        public OSDUTF8(byte[] value)
        {
            Type = OSDType.OSDUTF8;
            // Refuse to hold null pointers
            if (value != null)
                this.value = new osUTF8(value);
            else
                this.value = new osUTF8();
        }

        public OSDUTF8(string value)
        {
            Type = OSDType.OSDUTF8;
            // Refuse to hold null pointers
            if (value != null)
                this.value = new osUTF8(value);
            else
                this.value = new osUTF8();
        }

        public override bool AsBoolean()
        {
            if (osUTF8.IsNullOrEmpty(value))
                return false;

            if (value.Equals('0') || value.ACSIILowerEquals("false"))
                return false;

            return true;
        }

        public override int AsInteger()
        {
            if (Double.TryParse(value.ToString().AsSpan(), out double dbl))
                return (int)Math.Floor(dbl);
            else
                return 0;
        }

        public override uint AsUInteger()
        {
            if (Double.TryParse(value.ToString().AsSpan(), out double dbl))
                return (uint)Math.Floor(dbl);
            else
                return 0;
        }

        public override long AsLong()
        {
            if (Double.TryParse(value.ToString().AsSpan(), out double dbl))
                return (long)Math.Floor(dbl);
            else
                return 0;
        }

        public override ulong AsULong()
        {
            if (Double.TryParse(value.ToString().AsSpan(), out double dbl))
                return (ulong)Math.Floor(dbl);
            else
                return 0;
        }

        public override double AsReal()
        {
            if (Double.TryParse(value.ToString().AsSpan(), out double dbl))
                return dbl;
            else
                return 0d;
        }

        public override string AsString() { return value.ToString(); }
        public override byte[] AsBinary() { return value.ToArray(); }

        public override UUID AsUUID()
        {
            if (UUID.TryParse(value.ToString().AsSpan(), out UUID uuid))
                return uuid;
            else
                return UUID.Zero;
        }

        public override DateTime AsDate()
        {
            if (DateTime.TryParse(value.ToString().AsSpan(), out DateTime dt))
                return dt;
            else
                return Utils.Epoch;
        }

        public override Uri AsUri()
        {
            if (Uri.TryCreate(value.ToString(), UriKind.RelativeOrAbsolute, out Uri uri))
                return uri;
            else
                return null;
        }

        public override string ToString() { return AsString(); }
    }

    public sealed class OSDString : OSD
    {
        public readonly string value;

        public override OSD Copy() { return new OSDString(value); }

        public OSDString(string value)
        {
            Type = OSDType.String;
            // Refuse to hold null pointers
            this.value = value ?? string.Empty;
        }

        public override bool AsBoolean()
        {
            if (String.IsNullOrEmpty(value))
                return false;

            if (value.Equals("0") || value.Equals( "false", StringComparison.InvariantCultureIgnoreCase))
                return false;

            return true;
        }

        public override int AsInteger()
        {
            if (Double.TryParse(value, out double dbl))
                return (int)Math.Floor(dbl);
            else
                return 0;
        }

        public override uint AsUInteger()
        {
            if (Double.TryParse(value, out double dbl))
                return (uint)Math.Floor(dbl);
            else
                return 0;
        }

        public override long AsLong()
        {
            if (Double.TryParse(value, out double dbl))
                return (long)Math.Floor(dbl);
            else
                return 0;
        }

        public override ulong AsULong()
        {
            if (Double.TryParse(value, out double dbl))
                return (ulong)Math.Floor(dbl);
            else
                return 0;
        }

        public override double AsReal()
        {
            if (Double.TryParse(value, out double dbl))
                return dbl;
            else
                return 0d;
        }

        public override string AsString() { return value; }
        public override byte[] AsBinary() { return Encoding.UTF8.GetBytes(value); }

        public override UUID AsUUID()
        {
            if (UUID.TryParse(value.AsSpan(), out UUID uuid))
                return uuid;
            else
                return UUID.Zero;
        }

        public override DateTime AsDate()
        {
            if (DateTime.TryParse(value, out DateTime dt))
                return dt;
            else
                return Utils.Epoch;
        }

        public override Uri AsUri()
        {
            if (Uri.TryCreate(value, UriKind.RelativeOrAbsolute, out Uri uri))
                return uri;
            else
                return null;
        }

        public override string ToString() { return AsString(); }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDUUID : OSD
    {
        public readonly UUID value;

        public OSDUUID(UUID value)
        {
            Type = OSDType.UUID;
            this.value = value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override OSD Copy() { return new OSDUUID(value); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override bool AsBoolean() { return value.IsNotZero(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string AsString() { return value.ToString(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override UUID AsUUID() { return value; }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override byte[] AsBinary() { return value.GetBytes(); }
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override string ToString() { return AsString(); }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDDate : OSD
    {
        public readonly DateTime value;

        public OSDDate(DateTime value)
        {
            Type = OSDType.Date;
            this.value = value;
        }

        public override string AsString()
        {
            string format;
            if (value.Millisecond > 0)
                format = "yyyy-MM-ddTHH:mm:ss.ffZ";
            else
                format = "yyyy-MM-ddTHH:mm:ssZ";
            return value.ToUniversalTime().ToString(format);
        }

        public override int AsInteger()
        {
            return (int)Utils.DateTimeToUnixTime(value);
        }

        public override uint AsUInteger()
        {
            return Utils.DateTimeToUnixTime(value);
        }

        public override long AsLong()
        {
            return (long)Utils.DateTimeToUnixTime(value);
        }

        public override ulong AsULong()
        {
            return Utils.DateTimeToUnixTime(value);
        }

        public override byte[] AsBinary()
        {
            TimeSpan ts = value.ToUniversalTime() - new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
            return Utils.DoubleToBytes(ts.TotalSeconds);
        }

        public override OSD Copy() { return new OSDDate(value); }
        public override DateTime AsDate() { return value; }
        public override string ToString() { return AsString(); }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDUri : OSD
    {
        public readonly Uri value;

        public OSDUri(Uri value)
        {
            Type = OSDType.URI;
            this.value = value;
        }

        public override string AsString()
        {
            if (value != null)
            {
                if (value.IsAbsoluteUri)
                    return value.AbsoluteUri;
                else
                    return value.ToString();
            }
            return string.Empty;
        }

        public override OSD Copy() { return new OSDUri(value); }
        public override Uri AsUri() { return value; }
        public override byte[] AsBinary() { return Encoding.UTF8.GetBytes(AsString()); }
        public override string ToString() { return AsString(); }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDBinary : OSD
    {
        public readonly byte[] value;

        public OSDBinary(byte[] value)
        {
            Type = OSDType.Binary;
            if (value != null)
                this.value = value;
            else
                this.value = Array.Empty<byte>();
        }

        public OSDBinary(uint value)
        {
            Type = OSDType.Binary;
            this.value = new byte[]
            {
                (byte)((value >> 24) & 0xFF),
                (byte)((value >> 16) & 0xFF),
                (byte)((value >> 8) & 0xFF),
                (byte)(value& 0xFF)
            };
        }

        public OSDBinary(long value)
        {
            Type = OSDType.Binary;
            this.value = new byte[]
            {
                (byte)((value >> 56) & 0xFF),
                (byte)((value >> 48) & 0xFF),
                (byte)((value >> 40) & 0xFF),
                (byte)((value >> 32) & 0xFF),
                (byte)((value >> 24) & 0xFF),
                (byte)((value >> 16) & 0xFF),
                (byte)((value >> 8) & 0xFF),
                (byte)(value& 0xFF)
            };
        }

        public OSDBinary(ulong value)
        {
            Type = OSDType.Binary;
            this.value = new byte[]
            {
                (byte)((value >> 56) & 0xFF),
                (byte)((value >> 48) & 0xFF),
                (byte)((value >> 40) & 0xFF),
                (byte)((value >> 32) & 0xFF),
                (byte)((value >> 24) & 0xFF),
                (byte)((value >> 16) & 0xFF),
                (byte)((value >> 8) & 0xFF),
                (byte)(value& 0xFF)
            };
        }

        public override OSD Copy() { return new OSDBinary(value); }
        public override string AsString() { return Convert.ToBase64String(value); }
        public override byte[] AsBinary() { return value; }

        public override int AsInteger()
        {
            return ((value[0] << 24) | (value[1] << 16) | (value[2] << 8) |  (value[3] << 0));
        }

        public override uint AsUInteger()
        {
            return (uint)((value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3] << 0));}

        public override long AsLong()
        {
            return (long)(
                ((long)value[0] << 56) |
                ((long)value[1] << 48) |
                ((long)value[2] << 40) |
                ((long)value[3] << 32) |
                ((long)value[4] << 24) |
                ((long)value[5] << 16) |
                ((long)value[6] << 8) |
                ((long)value[7] << 0));
        }

        public override ulong AsULong()
        {
            return (ulong)(
                ((ulong)value[0] << 56) |
                ((ulong)value[1] << 48) |
                ((ulong)value[2] << 40) |
                ((ulong)value[3] << 32) |
                ((ulong)value[4] << 24) |
                ((ulong)value[5] << 16) |
                ((ulong)value[6] << 8) |
                ((ulong)value[7] << 0));
        }

        public override string ToString()
        {
            return Utils.BytesToHexString(value, null);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDMap : OSD, IDictionary<string, OSD>
    {
        public readonly Dictionary<string, OSD> dicvalue;

        public OSDMap()
        {
            Type = OSDType.Map;
            dicvalue = new Dictionary<string, OSD>();
        }

        public OSDMap(int capacity)
        {
            Type = OSDType.Map;
            dicvalue = new Dictionary<string, OSD>(capacity);
        }

        public OSDMap(Dictionary<string, OSD> value)
        {
            Type = OSDType.Map;
            if (value != null)
                this.dicvalue = value;
            else
                this.dicvalue = new Dictionary<string, OSD>();
        }

        public override string ToString()
        {
            return OSDParser.SerializeJsonString(this, true);
        }

        public override OSD Copy()
        {
            return new OSDMap(new Dictionary<string, OSD>(dicvalue));
        }

        #region IDictionary Implementation

        public int Count { get { return dicvalue.Count; } }
        public bool IsReadOnly { get { return false; } }
        public ICollection<string> Keys { get { return dicvalue.Keys; } }
        public ICollection<OSD> Values { get { return dicvalue.Values; } }

        public OSD this[string key]
        {
            get
            {
                if (dicvalue.TryGetValue(key, out OSD llsd))
                    return llsd;
                else
                    return new OSD();
            }
            set { dicvalue[key] = value; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool ContainsKey(string key)
        {
            return dicvalue.ContainsKey(key);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(string key, OSD llsd)
        {
            dicvalue.Add(key, llsd);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(KeyValuePair<string, OSD> kvp)
        {
            dicvalue.Add(kvp.Key, kvp.Value);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Remove(string key)
        {
            return dicvalue.Remove(key);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetValue(string key, out OSD llsd)
        {
            return dicvalue.TryGetValue(key, out llsd);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetOSDMap(string key, out OSDMap ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp) && tmp is OSDMap map)
            {
                ossd = map;
                return true;
            }
            ossd = null;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetOSDArray(string key, out OSDArray ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp) && tmp is OSDArray arr)
            {
                ossd = arr;
                return true;
            }
            ossd = null;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetBool(string key, out bool ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsBoolean();
                return true;
            }
            ossd = false;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetInt(string key, out int ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsInteger();
                return true;
            }
            ossd = 0;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetLong(string key, out long ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsLong();
                return true;
            }
            ossd = 0;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetFloat(string key, out float ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = (float)tmp.AsReal();
                return true;
            }
            ossd = 0f;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetDouble(string key, out double ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsReal();
                return true;
            }
            ossd = 0.0;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetString(string key, out string ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsString();
                return true;
            }
            ossd = string.Empty;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetUUID(string key, out UUID ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsUUID();
                return true;
            }
            ossd = UUID.Zero;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetDate(string key, out DateTime ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsDate();
                return true;
            }
            ossd = Utils.Epoch;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetUri(string key, out Uri ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsUri();
                return true;
            }
            ossd = null;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetBinary(string key, out byte[] ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsBinary();
                return true;
            }
            ossd = null;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetVector2(string key, out Vector2 ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsVector2();
                return true;
            }
            ossd = Vector2.Zero;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetVector3(string key, out Vector3 ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsVector3();
                return true;
            }
            ossd = Vector3.Zero;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetVector3d(string key, out Vector3d ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsVector3d();
                return true;
            }
            ossd = Vector3d.Zero;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetVector4(string key, out Vector4 ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsVector4();
                return true;
            }
            ossd = Vector4.Zero;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetQuat(string key, out Quaternion ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsQuaternion();
                return true;
            }
            ossd = Quaternion.Identity;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool TryGetColor4(string key, out Color4 ossd)
        {
            if (dicvalue.TryGetValue(key, out OSD tmp))
            {
                ossd = tmp.AsColor4();
                return true;
            }
            ossd = Color4.Black;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override void Clear()
        {
            dicvalue.Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(KeyValuePair<string, OSD> kvp)
        {
            // This is a bizarre function... we don't really implement it
            // properly, hopefully no one wants to use it
            return dicvalue.ContainsKey(kvp.Key);
        }

        public void CopyTo(KeyValuePair<string, OSD>[] array, int index)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Remove(KeyValuePair<string, OSD> kvp)
        {
            return dicvalue.Remove(kvp.Key);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public System.Collections.IDictionaryEnumerator GetEnumerator()
        {
            return dicvalue.GetEnumerator();
        }

        IEnumerator<KeyValuePair<string, OSD>> IEnumerable<KeyValuePair<string, OSD>>.GetEnumerator()
        {
            return null;
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return dicvalue.GetEnumerator();
        }

        #endregion IDictionary Implementation
    }

    /// <summary>
    /// 
    /// </summary>
    public sealed class OSDArray : OSD, IList<OSD>
    {
        public readonly List<OSD> value;

        public OSDArray()
        {
            Type = OSDType.Array;
            value = new List<OSD>();
        }

        public OSDArray(int capacity)
        {
            Type = OSDType.Array;
            value = new List<OSD>(capacity);
        }

        public OSDArray(List<OSD> value)
        {
            Type = OSDType.Array;
            if (value != null)
                this.value = value;
            else
                this.value = new List<OSD>();
        }

        public override byte[] AsBinary()
        {
            byte[] binary = new byte[value.Count];

            for (int i = 0; i < value.Count; i++)
                binary[i] = (byte)value[i].AsInteger();

            return binary;
        }

        public override long AsLong()
        {
            if (value.Count < 8)
                return 0;
            byte[] b = new byte[8];
            for (int i = 0; i < 8; i++)
                b[i] = (byte)value[i].AsInteger();
            return (
                ((long)b[0] << 56) |
                ((long)b[1] << 48) |
                ((long)b[2] << 40) |
                ((long)b[3] << 32) |
                ((long)b[4] << 24) |
                ((long)b[5] << 16) |
                ((long)b[6] << 8) |
                b[7]);
        }

        public override ulong AsULong()
        {
            if (value.Count < 8)
                return 0;
            byte[] b = new byte[8];
            for (int i = 0; i < 8; i++)
                b[i] = (byte)value[i].AsInteger();
            return (
                ((ulong)b[0] << 56) |
                ((ulong)b[1] << 48) |
                ((ulong)b[2] << 40) |
                ((ulong)b[3] << 32) |
                ((ulong)b[4] << 24) |
                ((ulong)b[5] << 16) |
                ((ulong)b[6] << 8) |
                b[7]);
        }

        public override int AsInteger()
        {
            if (value.Count < 4)
                return 0;
            byte[] by = new byte[4];
            for (int i = 0; i < 4; i++)
                by[i] = (byte)value[i].AsInteger();
            return (by[0] << 24) | (by[1] << 16) | (by[2] << 8) | by[3];
        }

        public override uint AsUInteger()
        {
            if (value.Count < 4)
                return 0;
            byte[] by = new byte[4];
            for (int i = 0; i < 4; i++)
                by[i] = (byte)value[i].AsInteger();
            return (uint)((by[0] << 24) | (by[1] << 16) | (by[2] << 8) | by[3]);
        }
        /*
        public override Vector2 AsVector2()
        {
            Vector2 vector = Vector2.Zero;

            if (this.Count == 2)
            {
                vector.X = (float)this[0].AsReal();
                vector.Y = (float)this[1].AsReal();
            }

            return vector;
        }

        public override Vector3 AsVector3()
        {
            Vector3 vector = Vector3.Zero;

            if (this.Count == 3)
            {
                vector.X = this[0].AsReal();
                vector.Y = this[1].AsReal();
                vector.Z = this[2].AsReal();
            }

            return vector;
        }

        public override Vector3d AsVector3d()
        {
            Vector3d vector = Vector3d.Zero;

            if (this.Count == 3)
            {
                vector.X = this[0].AsReal();
                vector.Y = this[1].AsReal();
                vector.Z = this[2].AsReal();
            }

            return vector;
        }

        public override Vector4 AsVector4()
        {
            Vector4 vector = Vector4.Zero;

            if (this.Count == 4)
            {
                vector.X = (float)this[0].AsReal();
                vector.Y = (float)this[1].AsReal();
                vector.Z = (float)this[2].AsReal();
                vector.W = (float)this[3].AsReal();
            }

            return vector;
        }

        public override Quaternion AsQuaternion()
        {
            Quaternion quaternion = Quaternion.Identity;

            if (this.Count == 4)
            {
                quaternion.X = (float)this[0].AsReal();
                quaternion.Y = (float)this[1].AsReal();
                quaternion.Z = (float)this[2].AsReal();
                quaternion.W = (float)this[3].AsReal();
            }

            return quaternion;
        }
        */
        public override Color4 AsColor4()
        {
            Color4 color = Color4.Black;

            if (this.Count == 4)
            {
                color.R = (float)this[0].AsReal();
                color.G = (float)this[1].AsReal();
                color.B = (float)this[2].AsReal();
                color.A = (float)this[3].AsReal();
            }

            return color;
        }

        public override OSD Copy()
        {
            return new OSDArray(new List<OSD>(value));
        }

        public override string ToString()
        {
            return OSDParser.SerializeJsonString(this, true);
        }

        #region IList Implementation

        public int Count { get { return value.Count; } }
        public bool IsReadOnly { get { return false; } }
        public OSD this[int index]
        {
            get { return value[index]; }
            set { this.value[index] = value; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int IndexOf(OSD llsd)
        {
            return value.IndexOf(llsd);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Insert(int index, OSD llsd)
        {
            value.Insert(index, llsd);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveAt(int index)
        {
            value.RemoveAt(index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Add(OSD llsd)
        {
            value.Add(llsd);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override void Clear()
        {
            value.Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(OSD llsd)
        {
            return value.Contains(llsd);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Contains(string element)
        {
            for (int i = 0; i < value.Count; i++)
            {
                if (value[i].Type == OSDType.String && value[i].AsString() == element)
                    return true;
            }

            return false;
        }

        public void CopyTo(OSD[] array, int index)
        {
            throw new NotImplementedException();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Remove(OSD llsd)
        {
            return value.Remove(llsd);
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return value.GetEnumerator();
        }

        IEnumerator<OSD> IEnumerable<OSD>.GetEnumerator()
        {
            return value.GetEnumerator();
        }

        #endregion IList Implementation
    }

    public partial class OSDParser
    {
        const string LLSD_BINARY_HEADER   = "<? llsd/binary";
        const string LLSD_NOTATION_HEADER = "<? llsd/notatio";
        const string LLSD_XML_HEADER      = "<llsd>";
        const string LLSD_XML_ALT_HEADER  = "<?xml";
        const string LLSD_XML_ALT2_HEADER = "<? llsd/xml";

        public static OSD Deserialize(byte[] data)
        {
            string header = Encoding.ASCII.GetString(data, 0, data.Length >= 15 ? 15 : data.Length);

            if (header.StartsWith(LLSD_XML_HEADER, StringComparison.InvariantCultureIgnoreCase) ||
                    header.StartsWith(LLSD_XML_ALT_HEADER, StringComparison.InvariantCultureIgnoreCase) ||
                    header.StartsWith(LLSD_XML_ALT2_HEADER, StringComparison.InvariantCultureIgnoreCase))
                return DeserializeLLSDXml(data);

            if (header.StartsWith(LLSD_NOTATION_HEADER, StringComparison.InvariantCultureIgnoreCase))
                return DeserializeLLSDNotation(data);

            if (header.StartsWith(LLSD_BINARY_HEADER, StringComparison.InvariantCultureIgnoreCase))
                return DeserializeLLSDBinary(data);

            return DeserializeJson(data);
        }

        public static OSD Deserialize(string data)
        {
            if (data.StartsWith(LLSD_XML_HEADER, StringComparison.InvariantCultureIgnoreCase) ||
                    data.StartsWith(LLSD_XML_ALT_HEADER, StringComparison.InvariantCultureIgnoreCase) ||
                    data.StartsWith(LLSD_XML_ALT2_HEADER, StringComparison.InvariantCultureIgnoreCase))
                return DeserializeLLSDXml(data);

            if (data.StartsWith(LLSD_NOTATION_HEADER, StringComparison.InvariantCultureIgnoreCase))
                return DeserializeLLSDNotation(data);

            if (data.StartsWith(LLSD_BINARY_HEADER, StringComparison.InvariantCultureIgnoreCase))
                return DeserializeLLSDBinary(Encoding.UTF8.GetBytes(data));

            return DeserializeJson(data);
        }

        public static OSD Deserialize(Stream stream)
        {
            if (stream.CanSeek)
            {
                byte[] headerData = new byte[15];
                stream.Read(headerData, 0, 15);
                stream.Seek(0, SeekOrigin.Begin);
                string header = Encoding.ASCII.GetString(headerData);

                if (header.StartsWith(LLSD_XML_HEADER) || header.StartsWith(LLSD_XML_ALT_HEADER) || header.StartsWith(LLSD_XML_ALT2_HEADER))
                    return DeserializeLLSDXml(stream);

                if (header.StartsWith(LLSD_NOTATION_HEADER))
                    return DeserializeLLSDNotation(stream);

                if (header.StartsWith(LLSD_BINARY_HEADER))
                    return DeserializeLLSDBinary(stream);

                return DeserializeJson(stream);
            }
            else
            {
                throw new OSDException("Cannot deserialize structured data from unseekable streams");
            }
        }
    }
}
