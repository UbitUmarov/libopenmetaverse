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
using System.Collections.Generic;
using System.IO;
using System.Net;
using System.Text;
using System.Xml;
using System.Reflection;
using OpenMetaverse;
using OpenMetaverse.Packets;
using OpenMetaverse.Messages;
using OpenMetaverse.StructuredData;
using OpenMetaverse.Interfaces;
using GridProxy;
using Nwc.XmlRpc;

namespace WinGridProxy
{
    #region Base Class
    internal abstract class Session
    {
        internal const string EmptyXml = "<?xml version=\"1.0\"?><Empty>XML representation of this item is not available.</Empty>";
        internal const string EmptyString = "String representation of this item is not available.";
        internal const string EmptyNotation = "Notation representation of this item is not available.";

        public Direction Direction { get; set; }
        public String Host { get; set; }
        public String Protocol { get; set; }
        public String Name { get; set; }
        public String ContentType { get; set; }
        public int Length { get; set; }
        public DateTime TimeStamp { get; set; }

        // listview specific stuff, not serialized or deserialized
        public bool Selected { get; set; }        
        public System.Drawing.Color BackColor { get; set; }

        public Session()
        {
            this.TimeStamp = DateTime.UtcNow;
            this.Host = this.Protocol = this.Name = string.Empty;
            this.Length = 0;
            this.ContentType = string.Empty;
        }

        public virtual string ToRawString(Direction direction)
        {
            return EmptyString;
        }

        public virtual byte[] ToBytes(Direction direction)
        {
            return Array.Empty<byte>();
        }

        public virtual string ToXml(Direction direction)
        {
            return EmptyXml;
        }

        public virtual string ToStringNotation(Direction direction)
        {
            return EmptyNotation;
        }

        public abstract string ToPrettyString(Direction direction);
        public abstract byte[] Serialize();
        public abstract Session Deserialize(byte[] bytes);
    }
    #endregion

    #region Packets
    internal sealed class SessionPacket : Session
    {
        public Packet Packet { get; set; }

        public SessionPacket() : base() { this.Protocol = "UDP"; }
        public SessionPacket(Packet packet, Direction direction, IPEndPoint endpoint, String contentType)
            : base()
        {
            this.Packet = packet;
            this.Name = packet.Type.ToString();
            this.Direction = direction;
            this.Host = String.Format("{0}:{1}", endpoint.Address, endpoint.Port);
            this.ContentType = contentType;
            this.Length = packet.Length;
            this.Protocol = "UDP";
        }

        public override string ToPrettyString(Direction direction)
        {
            if (direction == this.Direction)
                return PacketDecoder.PacketToString(this.Packet);
            else
                return string.Empty;
        }

        public override string ToRawString(Direction direction)
        {
            if (direction == this.Direction)
                return PacketDecoder.PacketToString(this.Packet);
            else
                return string.Empty;
        }

        public override byte[] ToBytes(Direction direction)
        {
            if (direction == this.Direction)
                return Packet.ToBytes();
            else
                return base.ToBytes(direction);
        }

        //public override string ToXml(Direction direction)
        //{
        //    if (direction == this.Direction)
        //        return Packet.ToXmlString(this.Packet);
        //    else
        //        return base.ToXml(direction);
        //}

        //public override string ToStringNotation(Direction direction)
        //{
        //    if (direction == this.Direction)
        //        return Packet.GetLLSD(this.Packet).ToString();
        //    else
        //        return base.ToStringNotation(direction);
        //}

        public override byte[] Serialize()
        {
            OSDMap map = new OSDMap(5);
            map["Name"] = OSD.FromString(this.Name);
            map["Host"] = OSD.FromString(this.Host);
            map["PacketBytes"] = OSD.FromBinary(this.Packet.ToBytes());
            map["Direction"] = OSD.FromInteger((int)this.Direction);
            map["ContentType"] = OSD.FromString(this.ContentType);

            return OpenMetaverse.Utils.StringToBytes(map.ToString());
        }

        public override Session Deserialize(byte[] bytes)
        {
            OSDMap map = (OSDMap)OSDParser.DeserializeLLSDNotation(OpenMetaverse.Utils.BytesToString(bytes));

            this.Host = map["Host"].AsString();
            this.Direction = (Direction)map["Direction"].AsInteger();
            this.ContentType = map["ContentType"].AsString();

            byte[] packetData = map["PacketBytes"].AsBinary();
            this.Length = packetData.Length;

            int packetEnd = packetData.Length;
            this.Packet = Packet.BuildPacket(packetData, ref packetEnd, null);
            this.Name = this.Packet.Type.ToString();
            return this;
        }
    }
    #endregion Packets

    #region Capabilities
    internal sealed class SessionCaps : Session
    {
        byte[] RequestBytes { get; set; }
        byte[] ResponseBytes { get; set; }
        WebHeaderCollection RequestHeaders { get; set; }
        WebHeaderCollection ResponseHeaders { get; set; }
        string FullUri { get; set; }
        string Method { get; set; }

        public SessionCaps() : base() { /*this.Protocol = "Caps";*/ }
        public SessionCaps(byte[] requestBytes, byte[] responseBytes,
            WebHeaderCollection requestHeaders, WebHeaderCollection responseHeaders,
            Direction direction, string uri, string capsKey, String proto, string fullUri, string meth)
            : base()
        {
            if (requestBytes != null)
                RequestBytes = requestBytes;
            else
                RequestBytes = Array.Empty<byte>();

            if (responseBytes != null)
                ResponseBytes = responseBytes;
            else
                ResponseBytes = Array.Empty<byte>();
            RequestHeaders = requestHeaders;
            ResponseHeaders = responseHeaders;
            Protocol = proto;
            FullUri = fullUri;
            Method = meth;

            Name = capsKey;
            Direction = direction;
            Host = uri;
            ContentType = (direction == Direction.Incoming) ? this.ResponseHeaders.Get("Content-Type") : this.RequestHeaders.Get("Content-Type");
            Length = (requestBytes != null) ? requestBytes.Length : 0;
            Length += (responseBytes != null) ? responseBytes.Length : 0;
        }

        public override string ToPrettyString(Direction direction)
        {
            try
            {
                if (direction == Direction.Incoming)
                {
                    if (this.ResponseBytes != null && this.ResponseBytes.Length > 0)
                    {
                        IMessage message = null;
                        OSD osd = OSDParser.Deserialize(this.ResponseBytes);

                        if (osd is OSDMap)
                        {
                            OSDMap data = (OSDMap)osd;

                            if (data.ContainsKey("body"))
                                message = OpenMetaverse.Messages.MessageUtils.DecodeEvent(this.Name, (OSDMap)data["body"]);
                            else
                                message = OpenMetaverse.Messages.MessageUtils.DecodeEvent(this.Name, data);

                            if (message != null)
                                return PacketDecoder.MessageToString(message, 0);
                            else
                                return "No Decoder for " + this.Name + Environment.NewLine +
                                       OSDParser.SerializeLLSDNotationFormatted(data) + Environment.NewLine +
                                       "Please report this at http://jira.openmetaverse.co Be sure to include the entire message.";
                        }
                    }
                }
                else
                {
                    if (this.RequestBytes != null && this.RequestBytes.Length > 0)
                    {
                        if (this.RequestBytes[0] == 60)
                        {
                            OSD osd = OSDParser.Deserialize(this.RequestBytes);
                            if (osd is OSDMap)
                            {
                                IMessage message = null;
                                OSDMap data = (OSDMap)osd;

                                if (data.ContainsKey("body"))
                                    message = MessageUtils.DecodeEvent(this.Name, (OSDMap)data["body"]);
                                else
                                    message = MessageUtils.DecodeEvent(this.Name, data);

                                if (message != null)
                                    return PacketDecoder.MessageToString(message, 0);
                                else
                                    return "No Decoder for " + this.Name + Environment.NewLine +
                                        OSDParser.SerializeLLSDNotationFormatted(data) + Environment.NewLine +
                                        "Please report this at http://jira.openmetaverse.co Be sure to include the entire message.";
                            }
                            else
                            {
                                return osd.ToString();
                            }
                        }
                        else
                        {
                            // this means its probably a script or asset using the uploader capability
                            // so we'll just return the raw bytes as a string
                            //if (this.RequestBytes[0] == 100)
                            //{
                            return Utils.BytesToString(this.RequestBytes);
                            //}
                        }
                    }
                    else
                    {
                        return string.Empty;
                    }
                }
            }
            catch { }
            return string.Empty;
        }

        public override string ToRawString(Direction direction)
        {
            try
            {
                if (direction == Direction.Incoming)
                {
                    if (this.ResponseBytes != null)
                    {
                        StringBuilder result = new StringBuilder();
                        foreach (String key in ResponseHeaders.Keys)
                        {
                            result.AppendFormat("{0} {1}" + Environment.NewLine, key, ResponseHeaders[key]);
                        }
                        result.AppendLine();
                        result.AppendLine(OpenMetaverse.Utils.BytesToString(this.ResponseBytes));
                        return result.ToString();
                    }
                    else
                        return string.Empty;
                }
                else
                {
                    if (this.RequestBytes != null)
                    {
                        StringBuilder result = new StringBuilder();
                        result.AppendFormat("{0}: {1}{2}", Method, FullUri, Environment.NewLine);
                        foreach (String key in RequestHeaders.Keys)
                        {
                            result.AppendFormat("{0}: {1}" + Environment.NewLine, key, RequestHeaders[key]);
                        }
                        result.AppendLine();
                        result.AppendLine(OpenMetaverse.Utils.BytesToString(this.RequestBytes));
                        return result.ToString();
                    }
                    else
                        return string.Empty;
                }
            }
            catch { }
            return string.Empty;
        }

        public override byte[] ToBytes(Direction direction)
        {
            if (direction == Direction.Incoming)
            {
                if (this.ResponseBytes != null)
                    return this.ResponseBytes;
                else
                    return base.ToBytes(direction);
            }
            else
            {
                if (this.RequestBytes != null)
                    return this.RequestBytes;
                else
                    return base.ToBytes(direction);
            }
        }

        public override string ToStringNotation(Direction direction)
        {
            try
            {
                if (direction == Direction.Incoming)
                {
                    if (this.ResponseBytes != null)
                        return BytesToOsd(this.ResponseBytes);
                    //return this.ResponseBytes;
                    else
                        return base.ToStringNotation(direction);
                }
                else
                {
                    if (this.RequestBytes != null)
                    {
                        return BytesToOsd(this.RequestBytes);
                    }
                    else
                        return base.ToStringNotation(direction);
                }
            }
            catch { }
            return string.Empty;
        }

        public override string ToXml(Direction direction)
        {
            try
            {
                if (direction == Direction.Incoming)
                {
                    if (this.ResponseBytes != null)
                        return BytesToXml(this.ResponseBytes);
                    else
                        return base.ToXml(direction);
                }
                else
                {
                    if (this.RequestBytes != null)
                        return BytesToXml(this.RequestBytes);
                    else
                        return base.ToXml(direction);
                }
            }
            catch { }
            return string.Empty;
        }

        // Sanity check the bytes are infact OSD
        private string BytesToOsd(byte[] bytes)
        {
            try
            {
                OSD osd = OSDParser.Deserialize(bytes);
                return OSDParser.SerializeLLSDNotationFormatted(osd);
            }
            catch (LitJson.JsonException)
            {
                // unable to decode as notation format
                return base.ToStringNotation(this.Direction);
            }            
        }

        // Sanity check the bytes are infact an XML
        private string BytesToXml(byte[] bytes)
        {
            String result = Utils.BytesToString(bytes);
            if (result.StartsWith("<?xml"))
                return result;
            else
                return base.ToXml(this.Direction);
        }


        public override byte[] Serialize()
        {
            OSDMap map = new OSDMap(5);
            map["Name"] = OSD.FromString(this.Name);
            map["Host"] = OSD.FromString(this.Host);
            map["RequestBytes"] = OSD.FromBinary(this.RequestBytes);
            map["ResponseBytes"] = OSD.FromBinary(this.ResponseBytes);
            map["Direction"] = OSD.FromInteger((int)this.Direction);
            map["ContentType"] = OSD.FromString(this.ContentType);
            map["Protocol"] = OSD.FromString(this.Protocol);

            OSDArray requestHeadersArray = new OSDArray();
            foreach (String key in this.RequestHeaders.Keys)
            {
                OSDMap rMap = new OSDMap(1);
                rMap[key] = OSD.FromString(this.RequestHeaders[key]);
                requestHeadersArray.Add(rMap);
            }
            if(requestHeadersArray.Count > 0)
                map["RequestHeaders"] = requestHeadersArray;

            OSDArray responseHeadersArray = new OSDArray();
            foreach (String key in this.ResponseHeaders.Keys)
            {
                OSDMap rMap = new OSDMap(1);
                rMap[key] = OSD.FromString(this.ResponseHeaders[key]);
                responseHeadersArray.Add(rMap);
            }
            if(responseHeadersArray.Count > 0)
                map["ResponseHeaders"] = responseHeadersArray;

            return OpenMetaverse.Utils.StringToBytes(map.ToString());
        }

        public override Session Deserialize(byte[] bytes)
        {
            OSDMap map = (OSDMap)OSDParser.DeserializeLLSDNotation(OpenMetaverse.Utils.BytesToString(bytes));

            this.Name = map["Name"].AsString();
            this.Host = map["Host"].AsString();
            this.RequestBytes = map["RequestBytes"].AsBinary();
            this.ResponseBytes = map["ResponseBytes"].AsBinary();
            this.Direction = (Direction)map["Direction"].AsInteger();
            this.Length = ResponseBytes.Length + RequestBytes.Length;
            this.ContentType = map["ContentType"].AsString();
            this.Protocol = map["Protocol"].AsString();

            this.RequestHeaders = new WebHeaderCollection();
            if (map.ContainsKey("RequestHeaders"))
            {
                OSDArray requestHeadersArray = (OSDArray)map["RequestHeaders"];
                for (int i = 0; i < requestHeadersArray.Count; i++)
                {
                    OSDMap rMap = (OSDMap)requestHeadersArray[i];
                    foreach (string key in rMap.Keys)
                    {
                        this.RequestHeaders.Add(key, rMap[key].AsString());
                    }
                }
            }

            this.ResponseHeaders = new WebHeaderCollection();
            if (map.ContainsKey("ResponseHeaders"))
            {
                OSDArray responseHeadersArray = (OSDArray)map["ResponseHeaders"];
                for (int i = 0; i < responseHeadersArray.Count; i++)
                {
                    OSDMap rMap = (OSDMap)responseHeadersArray[i];
                    foreach (string key in rMap.Keys)
                    {
                        this.ResponseHeaders.Add(key, rMap[key].AsString());
                    }
                }
            }

            return this;
        }
    }
    #endregion Capabilities

    #region Login
    internal sealed class SessionLogin : Session
    {
        private object Data { get; set; }
        private Dictionary<string,string> m_headers = null;
        //request, direction, comboBoxLoginURL.Text
        public SessionLogin() : base()
        {
            Protocol = "https";
        }
        public SessionLogin(object request, int rsize, Dictionary<string, string> headers, Direction direction, String url, String contentType)
            : base()
        {
            Data = request;
            Direction = direction;
            Host = url;
            ContentType = contentType;
            Name = (direction == Direction.Incoming) ? "Login Response" : "Login Request";
            Protocol = "https";
            if(rsize > 0)
                Length = rsize;
            else
                Length = ToRawString(Direction).Length;
            m_headers = headers;
        }

        public override string ToPrettyString(Direction direction)
        {
            if (direction == Direction)
            {
                return Data.ToString();
            }
            else
            {
                return string.Empty;
            }
        }

        public override string ToRawString(Direction direction)
        {
            if (direction == Direction)
            {
                StringBuilder sb = new StringBuilder(4096);
                if(m_headers != null)
                {
                    if(m_headers.ContainsKey("method"))
                    {
                        sb.Append(m_headers["method"]);
                        sb.Append(": ");
                        sb.Append(Host);
                        sb.AppendLine();
                    }
                    m_headers.Remove("host");
                    foreach (KeyValuePair<string,string> kvp in m_headers)
                    {
                        if(kvp.Key == "method")
                            continue;
                        sb.Append(kvp.Key);
                        sb.Append(' ');
                        sb.Append(kvp.Value);
                        sb.AppendLine();
                    }
                    sb.AppendLine();
                }
                using (MemoryStream ms = new MemoryStream(1024))
                {
                    using (XmlTextWriter xml = new XmlTextWriter(ms, new UTF8Encoding(false)))
                    {
                        if(Data is XmlRpcResponse)
                        {
                            var xrpcr = new XmlRpcResponseSerializer();
                            xrpcr.Serialize(xml, Data);
                        }
                        else
                        {
                            var xrpcr = new XmlRpcRequestSerializer();
                            xrpcr.Serialize(xml, Data);
                        }
                        xml.Flush();
                        sb.Append(UTF8Encoding.UTF8.GetString(ms.ToArray()));
                    }
                }
                return sb.ToString();
            }
            else
            {
                return string.Empty;
            }
        }
        public override string ToXml(Direction direction)
        {
            return base.ToXml(direction);

            //if (direction == this.Direction)
            //{
            //    return this.Data.ToString();
            //}
            //else
            //{
            //    return base.ToXml(direction);
            //}
        }

        public override byte[] ToBytes(Direction direction)
        {
            if (direction == this.Direction)
            {
                return OpenMetaverse.Utils.StringToBytes(this.Data.ToString());
            }
            else
            {
                return base.ToBytes(direction);
            }
        }

        public override byte[] Serialize()
        {
            OSDMap map = new OSDMap(6);
            map["Name"] = OSD.FromString(this.Name);
            map["Host"] = OSD.FromString(this.Host);
            map["Data"] = OSD.FromString(this.Data.ToString());
            map["Direction"] = OSD.FromInteger((int)this.Direction);
            map["ContentType"] = OSD.FromString(this.ContentType);
            map["Protocol"] = OSD.FromString(this.Protocol);

            return OpenMetaverse.Utils.StringToBytes(map.ToString());
        }

        public override Session Deserialize(byte[] bytes)
        {
            OSDMap map = (OSDMap)OSDParser.DeserializeLLSDNotation(OpenMetaverse.Utils.BytesToString(bytes));

            this.Name = map["Name"].AsString();
            this.Host = map["Host"].AsString();
            this.Data = map["Data"].AsString();
            this.Length = this.Data.ToString().Length;
            this.Direction = (Direction)map["Direction"].AsInteger();
            this.ContentType = map["ContentType"].AsString();
            this.Protocol = map["Protocol"].AsString();

            return this;
        }
    }
    #endregion Login

    #region EventQueue Messages
    internal class SessionEvent : Session
    {
        private byte[] ResponseBytes;
        private WebHeaderCollection ResponseHeaders;
        private string ExtraInfo;

        public SessionEvent() : base() { this.Protocol = "EventQ"; }
        public SessionEvent(byte[] responseBytes, WebHeaderCollection responseHeaders,
            String uri, String capsKey, String proto, string ExtraInfo)
            : base()
        {
            this.Protocol = proto;
            this.Direction = Direction.Incoming; // EventQueue Messages are always inbound from the simulator
            this.ResponseBytes = responseBytes;
            this.ResponseHeaders = responseHeaders;
            this.Host = uri;
            this.Name = capsKey;
            this.ContentType = responseHeaders.Get("Content-Type");
            var ContentLength = responseHeaders.Get("Content-Length");
            this.Length = ContentLength is null ? 0 : Int32.Parse (ContentLength);
            this.ExtraInfo = ExtraInfo;
        }

        public override string ToPrettyString(Direction direction)
        {
            if (direction == this.Direction)
            {
                IMessage message = null;
                OSD osd;
                try
                {
                    osd = OSDParser.Deserialize(this.ResponseBytes);
                }
                catch (Exception e)
                {
                    return "Decode failed";
                }
                OSDMap data = (OSDMap)osd;
                if (data.ContainsKey("body"))
                    message = MessageUtils.DecodeEvent(this.Name, (OSDMap)data["body"]);
                else
                    message = MessageUtils.DecodeEvent(this.Name, data);

                if (message != null)
                    return PacketDecoder.MessageToString(message, 0);
                else
                    return "No Decoder for " + this.Name + Environment.NewLine + osd.ToString();
            }
            else
            {
                return string.Empty;
            }
        }

        public override byte[] ToBytes(Direction direction)
        {
            if (direction == this.Direction)
            {
                return this.ResponseBytes;
            }
            else
            {
                return base.ToBytes(direction);
            }
        }

        public override string ToRawString(Direction direction)
        {

            if (direction == this.Direction)
            {
                StringBuilder result = new StringBuilder();
                foreach (String key in ResponseHeaders.Keys)
                {
                    //result.AppendFormat("{0} {1}" + Environment.NewLine, key, ResponseHeaders[key]);
                    result.AppendLine($"{key} {ResponseHeaders[key]}");
                }
                result.AppendLine();
                if (!string.IsNullOrEmpty(this.ExtraInfo))
                {
                    result.Append(this.ExtraInfo);
                    result.AppendLine();
                    result.AppendLine();
                }
                result.AppendLine(this.ToXml(direction));
                return result.ToString();
            }
            else
            {
                return string.Empty;
            }
        }

        public override string ToXml(Direction direction)
        {
            if (direction == this.Direction)
            {
                return OpenMetaverse.Utils.BytesToString(this.ResponseBytes);
            }
            else
            {
                return base.ToXml(direction);
            }
        }

        public override string ToStringNotation(Direction direction)
        {
            if (direction == this.Direction)
            {
                OSD osd = OSDParser.DeserializeLLSDXml(this.ResponseBytes);
                return osd.ToString();
            }
            else
            {
                return base.ToStringNotation(direction);
            }
        }

        public override byte[] Serialize()
        {
            OSDMap map = new OSDMap(7);
            map["Name"] = OSD.FromString(this.Name);
            map["Host"] = OSD.FromString(this.Host);
            map["ResponseBytes"] = OSD.FromBinary(this.ResponseBytes);
            map["Direction"] = OSD.FromInteger((int)this.Direction);
            map["ContentType"] = OSD.FromString(this.ContentType);
            map["Protocol"] = OSD.FromString(this.Protocol);

            OSDArray responseHeadersArray = new OSDArray();
            foreach (String key in this.ResponseHeaders.Keys)
            {
                OSDMap rMap = new OSDMap(1);
                rMap[key] = OSD.FromString(this.ResponseHeaders[key]);
                responseHeadersArray.Add(rMap);
            }
            map["ResponseHeaders"] = responseHeadersArray;

            return Utils.StringToBytes(map.ToString());
        }

        public override Session Deserialize(byte[] bytes)
        {
            OSDMap map = (OSDMap)OSDParser.DeserializeLLSDNotation(OpenMetaverse.Utils.BytesToString(bytes));

            this.Name = map["Name"].AsString();
            this.Host = map["Host"].AsString();
            this.ResponseBytes = map["ResponseBytes"].AsBinary();
            this.Direction = (Direction)map["Direction"].AsInteger();
            this.ContentType = map["ContentType"].AsString();
            this.Protocol = map["Protocol"].AsString();
            this.Length = ResponseBytes.Length;

            if (map.ContainsKey("ResponseHeaders"))
            {
                this.ResponseHeaders = new WebHeaderCollection();
                OSDArray responseHeadersArray = (OSDArray)map["ResponseHeaders"];
                for (int i = 0; i < responseHeadersArray.Count; i++)
                {
                    OSDMap rMap = (OSDMap)responseHeadersArray[i];
                    foreach (string key in rMap.Keys)
                    {
                        this.ResponseHeaders.Add(key, rMap[key].AsString());
                    }
                }
            }
            return this;
        }
    }
    #endregion
}
