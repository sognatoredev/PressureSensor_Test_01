import wave
import numpy as np
from scipy import signal
import struct
import os

def analyze_wav(filepath):
    print("=" * 70)
    print(f"파일: {filepath}")
    print("=" * 70)

    if not os.path.exists(filepath):
        print(f"  [오류] 파일을 찾을 수 없습니다: {filepath}")
        return

    # ── 1. 기본 정보 ────────────────────────────────────────────────────────
    with wave.open(filepath, 'rb') as wf:
        n_channels   = wf.getnchannels()
        sampwidth    = wf.getsampwidth()      # bytes per sample
        framerate    = wf.getframerate()
        n_frames     = wf.getnframes()
        raw          = wf.readframes(n_frames)

    bit_depth  = sampwidth * 8
    duration   = n_frames / framerate

    # PCM → numpy
    if bit_depth == 16:
        dtype  = np.int16
        pcm_max = 32768.0
    elif bit_depth == 8:
        dtype  = np.uint8
        pcm_max = 128.0
    elif bit_depth == 32:
        dtype  = np.int32
        pcm_max = 2147483648.0
    else:
        dtype  = np.int16
        pcm_max = 32768.0

    samples = np.frombuffer(raw, dtype=dtype).astype(np.float64)

    # 스테레오면 채널 분리 후 첫 번째 채널 사용
    if n_channels > 1:
        samples = samples[::n_channels]

    pcm_min  = int(samples.min())
    pcm_max_ = int(samples.max())
    rms      = np.sqrt(np.mean(samples ** 2))

    print(f"\n[1] 기본 정보")
    print(f"  샘플레이트  : {framerate} Hz")
    print(f"  채널        : {n_channels}")
    print(f"  비트깊이    : {bit_depth} bit")
    print(f"  프레임 수   : {n_frames}")
    print(f"  길이        : {duration:.3f} 초")
    print(f"  PCM min     : {pcm_min}")
    print(f"  PCM max     : {pcm_max_}")
    print(f"  PCM RMS     : {rms:.2f}")

    # ── 2. FFT 분석 ─────────────────────────────────────────────────────────
    N   = len(samples)
    win = np.hanning(N)
    fft_vals = np.fft.rfft(samples * win)
    freqs    = np.fft.rfftfreq(N, d=1.0 / framerate)

    magnitude = np.abs(fft_vals) * 2 / N          # 단측 스펙트럼 보정
    power     = magnitude ** 2

    # dB 변환 (0 나눔 방지)
    mag_db = 20 * np.log10(np.maximum(magnitude, 1e-12))

    print(f"\n[2] FFT 분석")
    print(f"  FFT 점수    : {N}")
    print(f"  주파수 해상도: {freqs[1]:.4f} Hz")

    # 주요 피크 Top 10 (전체)
    peak_indices = signal.find_peaks(magnitude, height=magnitude.max() * 0.01)[0]
    if len(peak_indices) == 0:
        peak_indices = np.argsort(magnitude)[::-1][:20]

    sorted_peaks = peak_indices[np.argsort(magnitude[peak_indices])[::-1]]
    top10 = sorted_peaks[:10]

    print(f"\n  [주요 피크 Top 10 - 전체 스펙트럼]")
    print(f"  {'순위':>4}  {'주파수(Hz)':>12}  {'진폭':>14}  {'dB':>10}")
    print(f"  {'-'*4}  {'-'*12}  {'-'*14}  {'-'*10}")
    for rank, idx in enumerate(top10, 1):
        print(f"  {rank:>4}  {freqs[idx]:>12.2f}  {magnitude[idx]:>14.4f}  {mag_db[idx]:>10.2f}")

    # 200~2500 Hz 대역 피크
    band_mask = (freqs >= 200) & (freqs <= 2500)
    band_freqs = freqs[band_mask]
    band_mag   = magnitude[band_mask]
    band_db    = mag_db[band_mask]

    if len(band_mag) > 0:
        band_peak_idx_local = signal.find_peaks(band_mag, height=band_mag.max() * 0.05)[0]
        if len(band_peak_idx_local) == 0:
            band_peak_idx_local = np.argsort(band_mag)[::-1][:10]
        band_sorted = band_peak_idx_local[np.argsort(band_mag[band_peak_idx_local])[::-1]]
        band_top = band_sorted[:10]

        print(f"\n  [200~2500 Hz 대역 피크 Top 10]")
        print(f"  {'순위':>4}  {'주파수(Hz)':>12}  {'진폭':>14}  {'dB':>10}")
        print(f"  {'-'*4}  {'-'*12}  {'-'*14}  {'-'*10}")
        for rank, idx in enumerate(band_top, 1):
            print(f"  {rank:>4}  {band_freqs[idx]:>12.2f}  {band_mag[idx]:>14.4f}  {band_db[idx]:>10.2f}")
    else:
        print(f"\n  [200~2500 Hz 대역] 해당 주파수 없음")

    # 대역별 에너지
    mask_low  = freqs <  200
    mask_mid  = (freqs >= 200) & (freqs <= 2500)
    mask_high = freqs >  2500

    energy_total = power.sum()
    energy_low   = power[mask_low].sum()
    energy_mid   = power[mask_mid].sum()
    energy_high  = power[mask_high].sum()

    def pct(e):
        return (e / energy_total * 100) if energy_total > 0 else 0.0

    print(f"\n  [대역별 에너지 비율]")
    print(f"  200 Hz 미만        : {pct(energy_low):>7.3f} %  (에너지: {energy_low:.4e})")
    print(f"  200 ~ 2500 Hz      : {pct(energy_mid):>7.3f} %  (에너지: {energy_mid:.4e})")
    print(f"  2500 Hz 초과       : {pct(energy_high):>7.3f} %  (에너지: {energy_high:.4e})")
    print(f"  전체 에너지        : {energy_total:.4e}")

    # ── 3. 노이즈 특성 ──────────────────────────────────────────────────────
    print(f"\n[3] 노이즈 특성")

    # SNR 추정: 200~2500 Hz 를 신호 대역으로 가정
    snr_linear = energy_mid / (energy_low + energy_high) if (energy_low + energy_high) > 0 else float('inf')
    snr_db_val = 10 * np.log10(snr_linear) if snr_linear > 0 else -np.inf
    print(f"  SNR (200~2500 Hz vs 나머지) : {snr_db_val:.2f} dB  (선형비: {snr_linear:.4f})")

    # 화이트노이즈 판별: 주파수 구간별 에너지 균등성 → 변동계수(CV)
    # 전체 스펙트럼을 20개 등간격 빈으로 나눔
    n_bins   = 20
    bin_size = len(power) // n_bins
    bin_energies = []
    for i in range(n_bins):
        start = i * bin_size
        end   = (i + 1) * bin_size if i < n_bins - 1 else len(power)
        bin_energies.append(power[start:end].mean())

    bin_energies = np.array(bin_energies)
    cv = bin_energies.std() / bin_energies.mean() if bin_energies.mean() > 0 else 0.0

    # 스펙트럼 기울기 (dB/decade) — 핑크노이즈 ≈ -10 dB/decade, 화이트노이즈 ≈ 0
    # 로그 주파수 축으로 선형 회귀
    valid = (freqs > 10) & (freqs < framerate / 2 * 0.9)
    log_f  = np.log10(freqs[valid])
    mag_db_valid = mag_db[valid]
    if len(log_f) > 2:
        slope, intercept = np.polyfit(log_f, mag_db_valid, 1)
    else:
        slope = 0.0

    print(f"  스펙트럼 기울기    : {slope:.2f} dB/decade")
    print(f"  에너지 분포 CV     : {cv:.4f}")

    if cv < 0.5 and abs(slope) < 5:
        noise_type = "화이트노이즈에 가까움 (평탄한 스펙트럼)"
    elif slope < -8:
        noise_type = "핑크/브라운 노이즈에 가까움 (저주파 에너지 집중)"
    elif slope > 5:
        noise_type = "고주파 에너지 집중 (블루노이즈 성향)"
    else:
        noise_type = "혼합 특성 (특정 주파수 집중 또는 구조적 신호)"

    print(f"  노이즈 특성 판별   : {noise_type}")

    # 에너지 분포 상세 (10개 빈)
    n_bins2  = 10
    bin_size2 = len(power) // n_bins2
    print(f"\n  [주파수 구간별 평균 에너지 (10구간)]")
    print(f"  {'구간':>6}  {'시작Hz':>10}  {'끝Hz':>10}  {'평균 에너지':>16}  {'dB':>8}")
    print(f"  {'-'*6}  {'-'*10}  {'-'*10}  {'-'*16}  {'-'*8}")
    for i in range(n_bins2):
        s = i * bin_size2
        e = (i + 1) * bin_size2 if i < n_bins2 - 1 else len(power)
        f_start = freqs[s]
        f_end   = freqs[min(e - 1, len(freqs) - 1)]
        avg_e   = power[s:e].mean()
        avg_db  = 10 * np.log10(avg_e) if avg_e > 0 else -np.inf
        print(f"  {i+1:>6}  {f_start:>10.1f}  {f_end:>10.1f}  {avg_e:>16.4e}  {avg_db:>8.2f}")

    print()


# ── 메인 실행 ─────────────────────────────────────────────────────────────────
files = [
    r"F:\COD0164.wav",
    r"F:\COD0165.wav",
    r"F:\COD0166.wav",
]

for f in files:
    analyze_wav(f)

print("=" * 70)
print("분석 완료")
print("=" * 70)
