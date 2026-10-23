using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

// Excel/guoliang wire format. No Unity APIs: safe on the UDP worker thread.
public static class ClientProtocol
{
    public const ushort Task = 0x0660, Control = 0x0662, Status = 0x0664, Stop = 0x0666, Situation = 0x0060;
    const int MaxBody = 1450;
    static readonly UTF8Encoding Utf8 = new UTF8Encoding(false, true);
    static readonly object stampGate = new object();
    static uint lastStamp;
    public static uint Timestamp(bool algorithm = true)
    {
        var now = DateTime.UtcNow;
        if (!algorithm) now = now.AddHours(8);
        uint stamp = (uint)(now.TimeOfDay.TotalMilliseconds * (algorithm ? 10 : 1));
        // Distinguish rapid same-tick Retry/reset snapshots (0.1ms task stamps).
        if (algorithm) lock (stampGate) { if (stamp <= lastStamp && lastStamp - stamp < 1000) stamp = lastStamp + 1; lastStamp = stamp; }
        return stamp;
    }
    static void U16(Stream s, int n) { s.WriteByte((byte)(n >> 8)); s.WriteByte((byte)n); }
    static void U32(Stream s, long n) { U16(s, (int)(n >> 16)); U16(s, (int)n); }
    public static int Read16(byte[] b, int p) => (b[p] << 8) | b[p + 1];
    public static uint Read32(byte[] b, int p) => ((uint)Read16(b, p) << 16) | (uint)Read16(b, p + 2);
    static MemoryStream Header(ushort kind, int length, uint stamp, int total = 0, int index = 1)
    {
        var s = new MemoryStream(); s.WriteByte(0); s.WriteByte(0); U16(s, length);
        U32(s, stamp); U16(s, 0); U16(s, kind);
        if (total > 0) { s.WriteByte((byte)total); s.WriteByte((byte)index); }
        return s;
    }
    public static ushort ParseHeader(byte[] data)
    {
        if (data.Length < 12 || Read16(data, 2) != data.Length || Read16(data, 8) != 0)
            throw new FormatException("Invalid client protocol header.");
        ushort kind = (ushort)Read16(data, 10);
        if (kind == Task || kind == Stop || kind == Status)
        {
            if (data.Length < 14 || data[0] != 0 || data[1] != 0 || data[12] < 1 || data[12] > 10
                || data[13] < 1 || data[13] > data[12] || data.Length - 14 > MaxBody)
                throw new FormatException("Invalid fragment header.");
        }
        else if (kind != Control && kind != Situation) throw new FormatException("Unknown information unit.");
        return kind;
    }
    public static List<byte[]> JsonPackets(JObject content, ushort kind, uint stamp)
    {
        if (kind != Task && kind != Stop && kind != Status) throw new ArgumentException("Expected JSON unit.");
        byte[] body = Utf8.GetBytes(content.ToString(Formatting.None));
        if (body.Length == 0 || body.Length > MaxBody * 10 - 2) throw new FormatException("JSON exceeds fragment budget.");
        int total = (body.Length + 2 + MaxBody - 1) / MaxBody, offset = 0;
        var result = new List<byte[]>();
        for (int i = 1; i <= total; i++)
        {
            int count = Math.Min(MaxBody - (i == 1 ? 2 : 0), body.Length - offset);
            using var s = Header(kind, 14 + count + (i == 1 ? 2 : 0), stamp, total, i);
            if (i == 1) U16(s, body.Length);
            s.Write(body, offset, count); offset += count; result.Add(s.ToArray());
        }
        return result;
    }
    public static byte[] SituationPacket(JObject r, uint stamp)
    {
        using var s = Header(Situation, 40, stamp);
        U16(s, (int)r["direction"]);
        U32(s, checked((int)Math.Round((double)r["latitude"] / 90 * 2147483648.0)));
        U32(s, checked((int)Math.Round((double)r["longitude"] / 180 * 2147483648.0)));
        U16(s, (int)r["platformId"]); U16(s, (int)r["sign"]);
        U16(s, r["targetCourse"].Type == JTokenType.Null ? 65535 : (int)Math.Round((double)r["targetCourse"]) % 360);
        U16(s, (int)r["height"]);
        double speed = (double)r["targetSpeed"];
        if (speed < -20 || speed > 200) throw new FormatException("Situation speed outside Excel range.");
        int raw = (int)Math.Round(speed * 10);
        // -1 is reserved for unknown speed; nearest representable reverse speed.
        if (raw == -1) raw = -2;
        U16(s, raw); U16(s, (int)r["type"]);
        s.WriteByte(r["targetTypeConfid"].Type == JTokenType.Null ? (byte)255 : (byte)r["targetTypeConfid"]);
        s.WriteByte(r["threatLevel"].Type == JTokenType.Null ? (byte)255 : (byte)r["threatLevel"]);
        U32(s, (long)r["batch"]); return s.ToArray();
    }
    public static JObject DecodeControl(byte[] data)
    {
        if (ParseHeader(data) != Control || data.Length != 20) throw new FormatException("Expected 20-byte control.");
        double course = Read16(data, 18) * .01;
        if (course >= 360) throw new FormatException("Invalid control course.");
        return new JObject { ["platCode"] = Read32(data, 12), ["speed"] = Read16(data, 16) * .01,
            ["course"] = course, ["timestamp"] = Read32(data, 4) };
    }
    public sealed class Reassembler
    {
        sealed class Entry { public DateTime created = DateTime.UtcNow; public int length = -1; public Dictionary<int, byte[]> parts = new(); }
        readonly Dictionary<string, Entry> pending = new();
        public JObject Add(byte[] data, string source)
        {
            ushort kind = ParseHeader(data);
            if (kind != Task && kind != Stop && kind != Status) throw new FormatException("Expected JSON frame.");
            foreach (var expired in pending.Where(p => (DateTime.UtcNow - p.Value.created).TotalSeconds > 3).Select(p => p.Key).ToArray())
                pending.Remove(expired);
            string key = source + "/" + kind + "/" + Read32(data, 4) + "/" + data[12];
            if (!pending.TryGetValue(key, out var entry))
            { if (pending.Count >= 128) pending.Remove(pending.Keys.First()); pending[key] = entry = new Entry(); }
            int index = data[13], offset = index == 1 ? 16 : 14;
            if (data.Length < offset) throw new FormatException("Missing contentLength.");
            if (index == 1)
            {
                int length = Read16(data, 14);
                if (length < 1 || length > MaxBody * 10 - 2 || (entry.length >= 0 && entry.length != length))
                    throw new FormatException("Invalid contentLength.");
                entry.length = length;
            }
            var piece = data.Skip(offset).ToArray();
            if (entry.parts.TryGetValue(index, out var previous) && !previous.SequenceEqual(piece))
            { pending.Remove(key); throw new FormatException("Conflicting fragment."); }
            entry.parts[index] = piece;
            if (entry.length < 0 || entry.parts.Count != data[12]) return null;
            pending.Remove(key);
            var bytes = Enumerable.Range(1, data[12]).SelectMany(i => entry.parts[i]).ToArray();
            if (bytes.Length != entry.length) throw new FormatException("Reassembled length mismatch.");
            return JObject.Parse(Utf8.GetString(bytes));
        }
    }
}
