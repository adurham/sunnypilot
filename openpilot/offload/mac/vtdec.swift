// vtdec.swift — offload Mac HEVC VideoToolbox decoder CLI (WS-B).
//
// stdin  (little-endian, repeated): [u32 nbytes][bytes][u64 pts][u32 flags]
//   flags bit0 = keyframe, bit1 = codec-config block (VPS/SPS/PPS, no frame out)
// stdout (little-endian, one record per decoded picture):
//   [u32 nbytes][bytes][u64 pts][u64 decode_ns]
//   bytes = DEVICE-GEOMETRY NV12 (420v biplanar, video range): stride=align(w,128),
//   Y plane y_height=align(h,32) rows at row*stride, UV plane uv_height=align(h/2,16)
//   rows at uv_offset + row*stride, buffer size = get_nv12_info(w,h) (OPEN-C1 fix).
//   nbytes is the FULL device VisionIPC buffer size (e.g. 4804608 for 1928x1208),
//   exactly what msgq create_buffers_with_sizes allocates on the device.
//
// CRITICAL: openpilot HEVC packs 3 slices per picture. VCL NALs are grouped into
// complete access units by first_slice_segment_in_pic_flag and submitted as ONE
// CMSampleBuffer; feeding slices individually makes VideoToolbox return -12909 on
// ~2/3 of frames. Parameter NALs (VPS/SPS/PPS) are harvested into the format
// description and are NOT part of the submitted sample (the reference harness
// did the same).
//
// Modes:
//   vtdec [--file F] [--format info|geom] [--params FILE] [--threads N] [--sw]
//   --format info : decode until the first picture, print
//                   {"width":W,"height":H,"pixelformat":"420v","pixelformat_code":N}
//                   to stdout, exit 0.
//   --format geom : decode until the first picture, print
//                   {"width":W,"height":H,"stride":S,"uv_offset":U,
//                    "y_height":YH,"uv_height":UVH,"size":SZ}
//                   to stdout, exit 0. Self-check: must equal
//                   openpilot/system/camerad/cameras/nv12_info.get_nv12_info(W,H).
//   --file F      : read the framed stream from F instead of stdin.
//   --params FILE : Annex-B file whose VPS/SPS/PPS seed the session.
//   --sw          : prefer software decode (default requires hardware).
//
// Decode errors emit nothing for that frame and log to stderr; the process keeps
// going. Non-zero exit only on a fatal setup error.

import Foundation
import VideoToolbox
import CoreMedia
import CoreVideo

// ---------- little-endian helpers ----------
@inline(__always) func rd32(_ p: UnsafeRawBufferPointer, _ o: Int) -> UInt32 {
    return UInt32(p[o]) | (UInt32(p[o + 1]) << 8) | (UInt32(p[o + 2]) << 16) | (UInt32(p[o + 3]) << 24)
}
@inline(__always) func rd64(_ p: UnsafeRawBufferPointer, _ o: Int) -> UInt64 {
    var v: UInt64 = 0
    for i in (0..<8).reversed() { v = (v << 8) | UInt64(p[o + i]) }
    return v
}
func wr32(_ out: inout Data, _ v: UInt32) {
    out.append(UInt8(v & 0xff)); out.append(UInt8((v >> 8) & 0xff))
    out.append(UInt8((v >> 16) & 0xff)); out.append(UInt8((v >> 24) & 0xff))
}
func wr64(_ out: inout Data, _ v: UInt64) {
    for i in 0..<8 { out.append(UInt8((v >> (8 * i)) & 0xff)) }
}
func elog(_ s: String) { FileHandle.standardError.write((s + "\n").data(using: .utf8)!) }

func monotonicNs() -> UInt64 {
    var tb = mach_timebase_info_data_t(); mach_timebase_info(&tb)
    return mach_absolute_time() * UInt64(tb.numer) / UInt64(tb.denom)  // codespell:ignore numer
}

// ---------- device NV12 geometry ----------
// OPEN-C1: VisionIPC buffers are allocated by camerad with VENUS NV12 geometry, NOT
// tight w*h*3/2. Emit exactly this layout so the consumer (modeld_v2) can read the
// buffer with device strides. MUST match openpilot/system/camerad/cameras/nv12_info.py
// get_nv12_info(width, height) -> (stride, y_height, uv_height, size); uv_offset = stride*y_height.
@inline(__always) func alignUp(_ val: Int, _ alignment: Int) -> Int {
    return ((val + alignment - 1) / alignment) * alignment
}

func nv12Info(_ width: Int, _ height: Int) -> (stride: Int, yHeight: Int, uvHeight: Int, size: Int) {
    let stride = alignUp(width, 128)
    let yHeight = alignUp(height, 32)
    let uvHeight = alignUp(height / 2, 16)

    // VENUS_BUFFER_SIZE for NV12
    let yPlane = stride * yHeight
    let uvPlane = stride * uvHeight + 4096
    var size = yPlane + uvPlane + max(16 * 1024, 8 * stride)
    size = alignUp(size, 4096)
    size += alignUp(width, 512) * 512  // kernel padding for non-aligned frames
    size = alignUp(size, 4096)

    return (stride, yHeight, uvHeight, size)
}

// ---------- Annex-B NAL parsing ----------
struct NAL { var start: Int; var end: Int; var type: Int; var firstSlice: Bool }

func parseNALs(_ d: [UInt8]) -> [NAL] {
    var out: [NAL] = []; let n = d.count; var i = 0
    var cs = -1, ct = -1, hl = 0
    func flush(_ e: Int) {
        if cs >= 0 {
            // first_slice_segment_in_pic_flag is the MSB of the byte after the 2-byte NAL header
            let flag = (hl + 2 < (e - cs)) ? ((d[cs + hl + 2] >> 7) & 1) == 1 : true
            out.append(NAL(start: cs + hl, end: e, type: ct, firstSlice: flag))
        }
    }
    while i < n - 3 {
        if d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 1 {
            flush(i); cs = i; hl = 3; ct = Int((d[i + 3] >> 1) & 0x3f); i += 4
        } else if d[i] == 0 && d[i + 1] == 0 && d[i + 2] == 0 && d[i + 3] == 1 {
            flush(i); cs = i; hl = 4; ct = Int((d[i + 4] >> 1) & 0x3f); i += 5
        } else { i += 1 }
    }
    flush(n); return out
}

// ---------- shared state ----------
let g = NSLock()
var submitNs: [UInt64: UInt64] = [:]      // pts -> submit ns
var delivered = 0, submitted = 0, submitErr = 0, decodeErr = 0
var lastErr: OSStatus = 0
var gInfoW = 0, gInfoH = 0
var gInfoStride = 0, gInfoUvOffset = 0, gInfoYH = 0, gInfoUVH = 0, gInfoSize = 0
var gPixelFormat: OSType = 0
let outLock = NSLock()

var gVPS: [UInt8]? = nil, gSPS: [UInt8]? = nil, gPPS: [UInt8]? = nil
var gFmt: CMFormatDescription? = nil
var gFmtKey = ""

func buildFormat() -> CMFormatDescription? {
    guard let v = gVPS, let s = gSPS, let p = gPPS else { return nil }
    let key = "\(v.count)\(s.count)\(p.count)"
    if gFmt != nil && gFmtKey == key { return gFmt }
    var holders: [UnsafeMutableBufferPointer<UInt8>] = []
    var ptrs: [UnsafePointer<UInt8>] = []; var sizes: [Int] = []
    for a in [v, s, p] {
        let b = UnsafeMutableBufferPointer<UInt8>.allocate(capacity: a.count); _ = b.initialize(from: a)
        holders.append(b); ptrs.append(UnsafePointer(b.baseAddress!)); sizes.append(a.count)
    }
    var fmt: CMFormatDescription?
    let st = CMVideoFormatDescriptionCreateFromHEVCParameterSets(
        allocator: kCFAllocatorDefault, parameterSetCount: 3,
        parameterSetPointers: &ptrs, parameterSetSizes: &sizes,
        nalUnitHeaderLength: 4, extensions: nil, formatDescriptionOut: &fmt)
    if st != noErr { elog("vtdec: format description failed \(st)"); return nil }
    gFmt = fmt; gFmtKey = key
    return fmt
}

let decodeCallback: VTDecompressionOutputCallback = { (_, srcRefcon, status, _, imageBuffer, _, _) in
    let cbT = monotonicNs()
    let pts = UInt64(bitPattern: Int64(Int(bitPattern: srcRefcon)))
    g.lock()
    let sub = submitNs[pts] ?? 0
    g.unlock()
    if status != 0 {
        g.lock(); decodeErr += 1; lastErr = status; g.unlock()
        elog("vtdec: decode error \(status) for pts \(pts) — frame dropped")
        return
    }
    guard let ib = imageBuffer else { return }
    CVPixelBufferLockBaseAddress(ib, .readOnly)
    defer { CVPixelBufferUnlockBaseAddress(ib, .readOnly) }
    let w = CVPixelBufferGetWidth(ib), h = CVPixelBufferGetHeight(ib)
    let fmt = CVPixelBufferGetPixelFormatType(ib)
    // Biplanar (420v): plane 0 = Y, plane 1 = interleaved CbCr. Row bytes are
    // per-plane and independent; never assume Y/UV contiguity.
    let planes = CVPixelBufferGetPlaneCount(ib)
    guard planes >= 1, let p0 = CVPixelBufferGetBaseAddressOfPlane(ib, 0) else { return }
    let r0 = CVPixelBufferGetBytesPerRowOfPlane(ib, 0)
    let ge = nv12Info(w, h)
    let stride = ge.stride, yHeight = ge.yHeight, uvHeight = ge.uvHeight
    let uvOffset = stride * yHeight
    // OPEN-C1: emit the DEVICE-geometry NV12 buffer (stride 2048 / uv_offset 2490368 /
    // size 4804608 for 1928x1208), i.e. exactly what camerad allocates via
    // create_buffers_with_sizes. Data(count:) is zero-initialized, so each row's
    // (stride - w) tail and the extra Y (yHeight - h) / UV (uvHeight - h/2) pad rows
    // are filled as required.
    var payload = Data(count: ge.size)
    let ySrc = p0.assumingMemoryBound(to: UInt8.self)
    payload.withUnsafeMutableBytes { dstRaw in
        let dst = dstRaw.bindMemory(to: UInt8.self)
        let yRows = min(h, yHeight)
        for y in 0..<yRows {
            let n = min(w, r0)
            if n > 0 { memcpy(dst.baseAddress! + y * stride, ySrc + y * r0, n) }
        }
    }
    if planes >= 2, let p1 = CVPixelBufferGetBaseAddressOfPlane(ib, 1) {
        let r1 = CVPixelBufferGetBytesPerRowOfPlane(ib, 1)
        let uvSrc = p1.assumingMemoryBound(to: UInt8.self)
        payload.withUnsafeMutableBytes { dstRaw in
            let dst = dstRaw.bindMemory(to: UInt8.self)
            let uvRows = min(h / 2, uvHeight)
            for y in 0..<uvRows {
                let n = min(w, r1)
                if n > 0 { memcpy(dst.baseAddress! + uvOffset + y * stride, uvSrc + y * r1, n) }
            }
        }
    }
    let decNs = cbT - sub
    g.lock()
    delivered += 1; gInfoW = w; gInfoH = h; gPixelFormat = fmt
    gInfoStride = stride; gInfoUvOffset = uvOffset; gInfoYH = yHeight; gInfoUVH = uvHeight; gInfoSize = ge.size
    g.unlock()
    if infoMode || geomMode {
        // Report dimensions / pixel format (or the full NV12 geometry) for the first
        // picture and stop; do not stream frame payloads to stdout in these modes.
        return
    }
    var rec = Data()
    wr32(&rec, UInt32(payload.count))
    rec.append(payload)
    wr64(&rec, pts)
    wr64(&rec, decNs)
    outLock.lock(); FileHandle.standardOutput.write(rec); outLock.unlock()
}

// ---------- main ----------
let args = CommandLine.arguments
var filePath: String? = nil
var paramsPath: String? = nil
var threads = 4
var useHW = true
var infoMode = false
var geomMode = false
var i = 1
while i < args.count {
    switch args[i] {
    case "--file": if i + 1 < args.count { filePath = args[i + 1] }; i += 2
    case "--params": if i + 1 < args.count { paramsPath = args[i + 1] }; i += 2
    case "--format":
        if i + 1 < args.count, args[i + 1] == "info" { infoMode = true }
        if i + 1 < args.count, args[i + 1] == "geom" { geomMode = true }
        i += 2
    case "--threads": if i + 1 < args.count { threads = Int(args[i + 1]) ?? 4 }; i += 2
    case "--sw": useHW = false; i += 1
    default: i += 1
    }
}

func harvest(_ data: [UInt8], _ nals: [NAL]) {
    g.lock()
    for n in nals {
        switch n.type {
        case 32: if gVPS == nil { gVPS = Array(data[n.start..<n.end]) }
        case 33: if gSPS == nil { gSPS = Array(data[n.start..<n.end]) }
        case 34: if gPPS == nil { gPPS = Array(data[n.start..<n.end]) }
        default: break
        }
    }
    g.unlock()
}

if let pp = paramsPath, let raw = FileManager.default.contents(atPath: pp) {
    let d = [UInt8](raw); harvest(d, parseNALs(d))
}

var session: VTDecompressionSession?
func ensureSession() -> VTDecompressionSession? {
    if let s = session { return s }
    guard let fmt = buildFormat() else { return nil }
    let dstAttrs: [CFString: Any] = [
        kCVPixelBufferPixelFormatTypeKey: kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange,
        kCVPixelBufferIOSurfacePropertiesKey: [:] as [CFString: Any],
        kCVPixelBufferMetalCompatibilityKey: true,
    ]
    var spec: [CFString: Any] = [:]
    if useHW {
        spec[kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder] = true
    } else {
        spec[kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder] = false
        spec[kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder] = false
    }
    var s: VTDecompressionSession?
    var cbRec = VTDecompressionOutputCallbackRecord(decompressionOutputCallback: decodeCallback, decompressionOutputRefCon: nil)
    let st = VTDecompressionSessionCreate(allocator: kCFAllocatorDefault, formatDescription: fmt,
        decoderSpecification: spec as CFDictionary, imageBufferAttributes: dstAttrs as CFDictionary,
        outputCallback: &cbRec, decompressionSessionOut: &s)
    if st != noErr { elog("vtdec: session create failed \(st)"); return nil }
    VTSessionSetProperty(s!, key: kVTDecompressionPropertyKey_RealTime, value: kCFBooleanTrue)
    VTSessionSetProperty(s!, key: kVTDecompressionPropertyKey_ThreadCount, value: NSNumber(value: threads))
    session = s
    return s
}

func makeSampleBuffer(_ nals: [NAL], _ data: [UInt8], _ fmt: CMFormatDescription, _ pts: UInt64) -> CMSampleBuffer? {
    var avcc = [UInt8]()
    for nal in nals {
        var l = UInt32(nal.end - nal.start).bigEndian
        withUnsafeBytes(of: &l) { avcc.append(contentsOf: $0) }
        avcc.append(contentsOf: data[nal.start..<nal.end])
    }
    let total = avcc.count
    guard total > 0 else { return nil }
    var bb: CMBlockBuffer?
    CMBlockBufferCreateWithMemoryBlock(allocator: kCFAllocatorDefault, memoryBlock: nil,
        blockLength: total, blockAllocator: kCFAllocatorDefault, customBlockSource: nil,
        offsetToData: 0, dataLength: total, flags: 0, blockBufferOut: &bb)
    guard let bbU = bb else { return nil }
    let rs = avcc.withUnsafeBytes { CMBlockBufferReplaceDataBytes(with: $0.baseAddress!, blockBuffer: bbU, offsetIntoDestination: 0, dataLength: total) }
    if rs != noErr { return nil }
    var sb: CMSampleBuffer?
    var timing = CMSampleTimingInfo(duration: CMTime(value: 20_000_000, timescale: 1_000_000_000),
                                    presentationTimeStamp: CMTime(value: Int64(bitPattern: pts), timescale: 1_000_000_000),
                                    decodeTimeStamp: .invalid)
    var sz = total
    let cs = CMSampleBufferCreateReady(allocator: kCFAllocatorDefault, dataBuffer: bbU, formatDescription: fmt,
        sampleCount: 1, sampleTimingEntryCount: 1, sampleTimingArray: &timing, sampleSizeEntryCount: 1, sampleSizeArray: &sz, sampleBufferOut: &sb)
    if cs != noErr { elog("vtdec: sample buffer failed \(cs)"); return nil }
    return sb
}

// Submit one access unit (the VCL NALs of a single picture) as one sample.
func submit(_ nals: [NAL], _ data: [UInt8], _ pts: UInt64) -> Bool {
    guard !nals.isEmpty else { return false }
    guard let s = ensureSession() else { elog("vtdec: no session/params yet — dropping pts \(pts)"); return false }
    guard let fmt = gFmt, let sbuf = makeSampleBuffer(nals, data, fmt, pts) else { return false }
    g.lock(); submitNs[pts] = monotonicNs(); submitted += 1; g.unlock()
    let refcon = UnsafeMutableRawPointer(bitPattern: Int(pts)) ?? UnsafeMutableRawPointer(bitPattern: 1)
    var info = VTDecodeInfoFlags()
    let r = VTDecompressionSessionDecodeFrame(s, sampleBuffer: sbuf, flags: [], frameRefcon: refcon, infoFlagsOut: &info)
    if r != noErr { g.lock(); submitErr += 1; g.unlock(); elog("vtdec: submit error \(r) for pts \(pts)"); return false }
    return true
}

// ---------- stream driver ----------
final class Reader {
    let fh: FileHandle
    init(_ fh: FileHandle) { self.fh = fh }
    func readExact(_ n: Int) -> [UInt8]? {
        if n == 0 { return [] }
        var buf = [UInt8](); buf.reserveCapacity(n)
        while buf.count < n {
            let d = fh.readData(ofLength: n - buf.count)
            if d.isEmpty { return nil }
            buf.append(contentsOf: d)
        }
        return buf
    }
}

let fh: FileHandle
if let fp = filePath {
    guard let f = FileHandle(forReadingAtPath: fp) else { elog("vtdec: cannot open \(fp)"); exit(1) }
    fh = f
} else {
    fh = FileHandle.standardInput
}
let reader = Reader(fh)

// Self-check probe (--format info|geom with a raw --params .hevc and no framed input):
// the params file is a whole Annex-B stream, so decode it in place to learn the
// decoded picture geometry. Streaming mode (no info/geom) never takes this branch.
var probeOnly = false
if (infoMode || geomMode) && filePath == nil, let pp = paramsPath, let raw = FileManager.default.contents(atPath: pp) {
    probeOnly = true
    let d = [UInt8](raw)
    let nals = parseNALs(d)
    harvest(d, nals)
    var aus: [[NAL]] = []
    var cur: [NAL] = []
    for n in nals where n.type <= 31 {
        if n.firstSlice && !cur.isEmpty { aus.append(cur); cur = [] }
        cur.append(n)
    }
    if !cur.isEmpty { aus.append(cur) }
    // Submit AUs until the first picture decodes (the stream may open mid-GOP, so the
    // leading non-IDR AUs are dropped by VT — expected). Drain asynchronously in chunks.
    for (k, au) in aus.enumerated() {
        if delivered > 0 { break }
        _ = submit(au, d, UInt64(k + 1))
        if k % 8 == 7, let s = session { VTDecompressionSessionWaitForAsynchronousFrames(s) }
    }
    if let s = session { VTDecompressionSessionWaitForAsynchronousFrames(s) }
}

// Per record: split VCL NALs into AUs by first_slice_segment_in_pic_flag and submit
// each complete AU. A record with no VCL is a codec-config block (harvest only).
// Since openpilot emits exactly one picture (3 slices) per EncodeData record, the
// common case yields a single AU per record; the loop still handles several.
var configOnly = true
while !probeOnly {
    guard let hdr = reader.readExact(4) else { break }
    let nbytes = Int(hdr.withUnsafeBytes { rd32($0, 0) })
    guard nbytes >= 0, nbytes < 64 * 1024 * 1024 else { elog("vtdec: bad nbytes \(nbytes)"); exit(1) }
    guard let body = reader.readExact(nbytes) else { elog("vtdec: truncated record (body)"); break }
    guard let meta = reader.readExact(12) else { elog("vtdec: truncated record (meta)"); break }
    let (pts, flags): (UInt64, UInt32) = meta.withUnsafeBytes { (rd64($0, 0), rd32($0, 8)) }
    let isConfig = (flags & 2) != 0

    let nals = parseNALs(body)
    harvest(body, nals)
    if isConfig {
        continue
    }

    var cur: [NAL] = []
    for n in nals {
        if n.type <= 31 {                        // VCL
            if n.firstSlice && !cur.isEmpty {
                _ = submit(cur, body, pts)
                cur = []
            }
            cur.append(n)
        }
        // Params and other NALs are harvested for the format description only.
    }
    if !cur.isEmpty {
        _ = submit(cur, body, pts)
        if !configOnly { }
    } else if !nals.isEmpty && !isConfig {
        // No VCL in a non-config record: nothing to decode.
        elog("vtdec: record with no VCL NALs (pts \(pts)) — skipped")
    }
    if submitted > 0 { configOnly = false }

    if (infoMode || geomMode) && delivered > 0 { break }
}

if let s = session {
    VTDecompressionSessionWaitForAsynchronousFrames(s)
}

g.lock()
let d = delivered, s = submitted, se = submitErr, de = decodeErr
let le = lastErr, pw = gInfoW, ph = gInfoH, pf = gPixelFormat
let pStride = gInfoStride, pUvOffset = gInfoUvOffset, pYH = gInfoYH, pUVH = gInfoUVH, pSize = gInfoSize
g.unlock()

if infoMode {
    if delivered > 0 {
        print("{\"width\":\(pw),\"height\":\(ph),\"pixelformat\":\"420v\",\"pixelformat_code\":\(Int(pf))}")
        exit(0)
    }
    elog("vtdec: no frame decoded for --format info")
    exit(1)
}
if geomMode {
    if delivered > 0 {
        print("{\"width\":\(pw),\"height\":\(ph),\"stride\":\(pStride),\"uv_offset\":\(pUvOffset),"
            + "\"y_height\":\(pYH),\"uv_height\":\(pUVH),\"size\":\(pSize)}")
        exit(0)
    }
    elog("vtdec: no frame decoded for --format geom")
    exit(1)
}
if submitted == 0 {
    elog("vtdec: no frames submitted (missing params? bad input?)")
    exit(1)
}
elog(String(format: "vtdec: submitted=%d delivered=%d submit_err=%d decode_err=%d last_err=%d dims=%dx%d fmt=%08x",
            s, d, se, de, le, pw, ph, pf))
exit(0)
