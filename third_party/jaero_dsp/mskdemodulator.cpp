/*
 * MskDemodulator -- JAERO's continuous MSK demodulator for P-channel.
 * Ported from github.com/jontio/JAERO, Qt-stripped to pure C++ with C callback.
 * SPDX-License-Identifier: MIT (JAERO original)
 *
 * This port uses the ORIGINAL JAERO processAudio() monolithic loop exactly
 * as it appears in JAERO desktop — the path that is known to decode when
 * fed real-valued audio centered at freq_center Hz. feedIQ is a thin
 * wrapper: mix IQ to int16 audio at freq_center Hz, then feedAudio.
 */

#include "mskdemodulator.h"
#include "coarsefreqestimate.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

static inline int qRoundI(double v) { return (int)(v + (v >= 0 ? 0.5 : -0.5)); }

MskDemodulator::MskDemodulator()
{
    soft_bits_cb = NULL;
    soft_bits_user = NULL;
    sigstat_cb = NULL;
    sigstat_user = NULL;
    sigstat_last = true;

    afc = false;
    sql = false;
    cpuReduce = false;

    Fs = 48000;
    freq_center = 1000.0;
    lockingbw = 900;
    fb = 600;

    signalthreshold = 0.5;

    SamplesPerSymbol = Fs / fb;

    scatterpointtype = SPT_constellation;

    bc.SetInNumberOfBits(1);
    bc.SetOutNumberOfBits(8);

    matchedfilter_re = new FIR(2 * SamplesPerSymbol);
    matchedfilter_im = new FIR(2 * SamplesPerSymbol);
    for (int i = 0; i < 2 * SamplesPerSymbol; i++) {
        matchedfilter_re->FIRSetPoint(i, sin(M_PI * i / (2.0 * SamplesPerSymbol)) / (2.0 * SamplesPerSymbol));
        matchedfilter_im->FIRSetPoint(i, sin(M_PI * i / (2.0 * SamplesPerSymbol)) / (2.0 * SamplesPerSymbol));
    }

    agc = new AGC(1, Fs);
    ebnomeasure = new MSKEbNoMeasure(0.5 * Fs);
    pointmean = new MovingAverage(100);

    mixer_center.SetFreq(freq_center, Fs);
    mixer2.SetFreq(freq_center, Fs);
    st_osc.SetFreq(fb / 2, Fs);

    bbnfft = pow(2, 14);
    bbcycbuff.resize(bbnfft);
    bbcycbuff_ptr = 0;
    bbtmpbuff.resize(bbnfft);

    spectrumnfft = pow(2, 13);
    spectrumcycbuff.resize(spectrumnfft);
    spectrumcycbuff_ptr = 0;
    spectrumtmpbuff.resize(spectrumnfft);

    pointbuff.resize(100);
    pointbuff_ptr = 0;
    mse = 10.0;
    msema = new MovingAverage(600);

    lastindex = 0;
    singlepointphasevector.resize(1);

    marg = new MovingAverage(80);
    dt.setLength(40);

    RxDataBits.reserve(100);

    coarsefreqestimate = new CoarseFreqEstimate();
    coarsefreqestimate->setSettings(14, lockingbw, fb, Fs);

    dcd = false;
    correctionfactor = 1.0;
    coarseCounter = 0;
    feediq_phase = 0.0;
    freqest_countdown = 4;
}

MskDemodulator::~MskDemodulator()
{
    delete matchedfilter_re;
    delete matchedfilter_im;
    delete agc;
    delete msema;
    delete ebnomeasure;
    delete pointmean;
    delete marg;
    delete coarsefreqestimate;
}

void MskDemodulator::setSettings(Settings s)
{
    last_applied_settings = s;
    Fs = s.Fs;
    lockingbw = s.lockingbw;
    fb = s.fb;
    freq_center = s.freq_center;
    if (freq_center > (Fs / 2.0 - lockingbw / 2.0))
        freq_center = Fs / 2.0 - lockingbw / 2.0;
    signalthreshold = s.signalthreshold;

    SamplesPerSymbol = (int)(Fs / fb);
    bbnfft = pow(2, s.coarsefreqest_fft_power);
    bbcycbuff.assign(bbnfft, cpx_type(0, 0));
    bbcycbuff_ptr = 0;
    bbtmpbuff.resize(bbnfft);
    coarsefreqestimate->setSettings(s.coarsefreqest_fft_power, lockingbw, fb, Fs);

    mixer_center.SetFreq(freq_center, Fs);
    mixer2.SetFreq(freq_center, Fs);
    st_osc.SetFreq(fb / 2, Fs);

    delete matchedfilter_re;
    delete matchedfilter_im;
    matchedfilter_re = new FIR(2 * SamplesPerSymbol);
    matchedfilter_im = new FIR(2 * SamplesPerSymbol);
    for (int i = 0; i < 2 * SamplesPerSymbol; i++) {
        matchedfilter_re->FIRSetPoint(i, sin(M_PI * i / (2.0 * SamplesPerSymbol)) / (2.0 * SamplesPerSymbol));
        matchedfilter_im->FIRSetPoint(i, sin(M_PI * i / (2.0 * SamplesPerSymbol)) / (2.0 * SamplesPerSymbol));
    }

    delete agc;
    agc = new AGC(1, Fs);

    delete ebnomeasure;
    ebnomeasure = new MSKEbNoMeasure(0.5 * Fs);

    pointbuff.assign(100, cpx_type(0, 0));
    pointbuff_ptr = 0;
    mse = 10.0;
    trackmode = false;

    lastindex = 0;

    st_iir_resonator.a.resize(3);
    st_iir_resonator.b.resize(3);

    /* Symbol timing resonator, designed for the ACTUAL sample rate.
     * JAERO baked coefficient tables for Fs of exactly 48000 (and a 12 kHz
     * fallback); our DDC delivers e.g. 48761.9 Hz (2.048 MHz / 42), which
     * silently selected the 12 kHz table and put the resonator at ~1218 Hz
     * instead of the fb/2 timing tone — the timing error signal was a
     * phase-shifted harmonic. Generic design: poles at radius r, angle
     * 2*pi*(fb/2)/Fs; bandwidth (1-r)*Fs/pi matches JAERO's intended
     * 2 Hz (600 baud) / 4 Hz (1200 baud); b0 = (1-r^2)/2 (peak gain ~1). */
    correctionfactor = (fb >= 1200) ? 0.6 : 1.0;
    ee = 0.025;
    {
        double bw_hz = (fb >= 1200) ? 4.0 : 2.0;
        double r = 1.0 - (M_PI * bw_hz / Fs);
        double th = 2.0 * M_PI * (fb / 2.0) / Fs;
        st_iir_resonator.a[0] = 1.0;
        st_iir_resonator.a[1] = -2.0 * r * std::cos(th);
        st_iir_resonator.a[2] = r * r;
        st_iir_resonator.b[0] = (1.0 - r * r) / 2.0;
        st_iir_resonator.b[1] = 0;
        st_iir_resonator.b[2] = -(1.0 - r * r) / 2.0;
    }
    st_iir_resonator.init();

    delete marg;
    marg = new MovingAverage(SamplesPerSymbol);
    dt.setLength(SamplesPerSymbol / 2);
    delayedsmpl.setLength(SamplesPerSymbol);
    delayt8.setdelay(SamplesPerSymbol / 2.0);
    coarseCounter = 0;

    /* Carrier tracking loop design (satdump costas_loop.cpp formula).
     * JAERO's original P+I pair (12deg / 0.12 Hz per update) has an
     * integrator ~35x too weak for its proportional gain (zeta ~2.1):
     * with a drifting SDR LO the loop limit-cycles between phase slips
     * instead of settling. A designed pair with zeta=1/sqrt(2) tracks a
     * ~1 Hz/s warmup ramp with sub-millirad steady-state error.
     * Bandwidths in Hz of loop noise bandwidth at the symbol-decision
     * update rate (fb/2): wide for acquisition pull-in from the coarse
     * estimate, narrow for tracking jitter. */
    carrier_upd_rate = fb / 2.0;
    auto design_loop = [&](double bn_hz, double &alpha, double &beta) {
        double damping = std::sqrt(2.0) / 2.0;
        double w = 2.0 * M_PI * bn_hz / carrier_upd_rate;
        double denom = 1.0 + 2.0 * damping * w + w * w;
        alpha = (4.0 * damping * w) / denom;
        beta = (4.0 * w * w) / denom;
    };
    /* Defaults benchmarked on real captures (see DSP_PLL_NOTES.md):
     * 3 Hz acquisition / 1.2 Hz tracking noise bandwidth, timing NCO
     * frequency-learning gain 3e-5. Env vars are tuning overrides for
     * headless benchmarks only. */
    double bn_acq = 3.0, bn_trk = 1.2;
    if (const char *e = getenv("MSK_BN_ACQ")) bn_acq = atof(e);
    if (const char *e = getenv("MSK_BN_TRK")) bn_trk = atof(e);
    design_loop(bn_acq, ct_alpha_acq, ct_beta_acq);
    design_loop(bn_trk, ct_alpha_trk, ct_beta_trk);
    st_gain_acq = 0.05;
    st_gain_trk = 0.003;
    st_freq_gain = 3e-5;
    if (const char *e = getenv("MSK_ST_ACQ")) st_gain_acq = atof(e);
    if (const char *e = getenv("MSK_ST_TRK")) st_gain_trk = atof(e);
    if (const char *e = getenv("MSK_ST_FREQ")) st_freq_gain = atof(e);
}

void MskDemodulator::invalidatesettings() { Fs = -1; fb = -1; }
void MskDemodulator::setAFC(bool s) { afc = s; }
void MskDemodulator::setSQL(bool s) { sql = s; }
void MskDemodulator::setCPUReduce(bool s) { cpuReduce = s; }
void MskDemodulator::setScatterPointType(ScatterPointType t) { scatterpointtype = (int)t; }
double MskDemodulator::getCurrentFreq() { return mixer_center.GetFreqHz(); }

int MskDemodulator::get_audio_snapshot(double *out, int max_samples)
{
    if (!out || max_samples <= 0) return 0;
    int n = (int)spectrumcycbuff.size();
    if (n > max_samples) n = max_samples;
    int start = spectrumcycbuff_ptr % (int)spectrumcycbuff.size();
    for (int i = 0; i < n; i++) {
        int idx = (start + i) % (int)spectrumcycbuff.size();
        out[i] = spectrumcycbuff[idx];
    }
    return n;
}

int MskDemodulator::get_constellation_snapshot(double *out, int max_pairs)
{
    if (!out || max_pairs <= 0) return 0;
    int cap = (int)pointbuff.size();
    if (cap <= 0) return 0;
    int n = cap;
    if (n > max_pairs) n = max_pairs;
    /* pointbuff is written by the DSP thread; we do an unlocked read
     * since a half-updated point renders as a misplaced dot at worst. */
    for (int i = 0; i < n; i++) {
        cpx_type p = pointbuff[i];
        out[i * 2]     = p.real();
        out[i * 2 + 1] = p.imag();
    }
    return n;
}

void MskDemodulator::setManualTune(double audio_hz)
{
    if (audio_hz < 500.0) audio_hz = 500.0;
    if (audio_hz > Fs / 2.0 - 500.0) audio_hz = Fs / 2.0 - 500.0;
    mixer_center.SetFreq(audio_hz, Fs);
    mixer2.SetFreq(audio_hz, Fs);
    if (coarsefreqestimate) coarsefreqestimate->bigchange();
    for (size_t j = 0; j < bbcycbuff.size(); j++)
        bbcycbuff[j] = cpx_type(0, 0);
}

void MskDemodulator::setSoftBitsCallback(msk_soft_bits_cb cb, void *user)
{
    soft_bits_cb = cb;
    soft_bits_user = user;
}

void MskDemodulator::DCDstatSlot(bool _dcd)
{
    dcd = _dcd;
}

void MskDemodulator::CenterFreqChangedSlot(double f)
{
    if (f < 0.75 * fb) f = 0.75 * fb;
    if (f > (Fs / 2.0 - 0.75 * fb)) f = Fs / 2.0 - 0.75 * fb;
    mixer_center.SetFreq(f, Fs);
    if (afc) mixer2.SetFreq(mixer_center.GetFreqHz());
    if ((mixer2.GetFreqHz() - mixer_center.GetFreqHz()) > (lockingbw / 2.0))
        mixer2.SetFreq(mixer_center.GetFreqHz() + (lockingbw / 2.0));
    if ((mixer2.GetFreqHz() - mixer_center.GetFreqHz()) < (-lockingbw / 2.0))
        mixer2.SetFreq(mixer_center.GetFreqHz() - (lockingbw / 2.0));
    for (size_t j = 0; j < bbcycbuff.size(); j++)
        bbcycbuff[j] = cpx_type(0, 0);
}

void MskDemodulator::FreqOffsetEstimateSlot(double freq_offset_est)
{
    /* Was static in JAERO (single-instance). Must be per-instance. */
    int &countdown = this->freqest_countdown;
    if ((mse > signalthreshold) &&
        (fabs(mixer2.GetFreqHz() - (mixer_center.GetFreqHz() + freq_offset_est)) > 0.0)) {
        mixer2.SetFreq(mixer_center.GetFreqHz() + freq_offset_est);
    }
    if (afc && dcd && fabs(mixer2.GetFreqHz() - mixer_center.GetFreqHz()) > 2.0) {
        if (countdown > 0) countdown--;
        else {
            mixer_center.SetFreq(mixer2.GetFreqHz());
            if (mixer_center.GetFreqHz() < lockingbw / 2.0)
                mixer_center.SetFreq(lockingbw / 2.0);
            if (mixer_center.GetFreqHz() > (Fs / 2.0 - lockingbw / 2.0))
                mixer_center.SetFreq(Fs / 2.0 - lockingbw / 2.0);
            coarsefreqestimate->bigchange();
            for (size_t j = 0; j < bbcycbuff.size(); j++)
                bbcycbuff[j] = cpx_type(0, 0);
        }
    } else countdown = 4;

    /* Signal status edge callback — matches JAERO's SignalStatus signal */
    if (sigstat_cb) {
        bool good = (mse < signalthreshold);
        if (good != sigstat_last) {
            sigstat_cb(good, sigstat_user);
            sigstat_last = good;
        }
    }
}

/* ORIGINAL JAERO processAudio — monolithic, unmodified. Operates on int16
 * mono audio at Fs with signal centered at freq_center Hz. */
void MskDemodulator::processAudio(const short *ptr, int numofsamples)
{
    for (int i = 0; i < numofsamples; i++) {
        double dval = (double)(*ptr) / 32768.0;

        spectrumcycbuff[spectrumcycbuff_ptr] = dval;
        spectrumcycbuff_ptr++;
        spectrumcycbuff_ptr %= spectrumnfft;

        if (coarseCounter >= Fs || !cpuReduce) {
            bbcycbuff[bbcycbuff_ptr] = mixer_center.WTCISValue() * dval;
            bbcycbuff_ptr++;
            bbcycbuff_ptr %= bbnfft;
            if (bbcycbuff_ptr % (cpuReduce ? bbnfft : bbnfft / 4) == 0) {
                for (int j = 0; j < bbnfft; j++) {
                    bbtmpbuff[j] = bbcycbuff[bbcycbuff_ptr];
                    bbcycbuff_ptr++;
                    bbcycbuff_ptr %= bbnfft;
                }
                coarsefreqestimate->ProcessBasebandData(bbtmpbuff);
                FreqOffsetEstimateSlot(coarsefreqestimate->getFreqOffsetEst());
                coarseCounter = 0;
            }
        }
        coarseCounter++;

        cpx_type cval = mixer2.WTCISValue() * dval;
        cpx_type sig2 = cpx_type(
            matchedfilter_re->FIRUpdateAndProcess(cval.real()),
            matchedfilter_im->FIRUpdateAndProcess(cval.imag()));

        double dabval = std::sqrt(sig2.real() * sig2.real() + sig2.imag() * sig2.imag());

        ebnomeasure->Update(dabval);
        sig2 *= agc->Update(dabval);

        double abval = std::sqrt(sig2.real() * sig2.real() + sig2.imag() * sig2.imag());
        if (abval > 2.84) sig2 = (2.84 / abval) * sig2;

        cpx_type _pt_d = delayedsmpl.update_dont_touch(sig2);
        cpx_type pt_msk = cpx_type(sig2.real(), _pt_d.imag());

        double st_eta = st_iir_resonator.update(std::abs(pt_msk));
        cpx_type st_m1 = cpx_type(st_eta, -delayt8.update(st_eta));
        cpx_type st_out = st_osc.WTCISValue() * st_m1;

        double st_angle_error = std::arg(st_out);
        double weighting = fabs(tanh(st_angle_error));

        /* Acquisition/tracking gain scheduling. JAERO switches on AeroL's
         * DCD, but frame sync often arrives while the constellation is
         * still poor (AeroL correlates soft bits, tolerating high MSE) —
         * dropping the gains 17x at that point makes convergence crawl for
         * tens of seconds. Gate on measured constellation quality instead,
         * with hysteresis so the switch doesn't chatter around the
         * threshold. (satdump avoids this trap by running one designed
         * loop; here we keep JAERO's two-mode structure but fix the gate.) */
        if (mse < 0.8 * signalthreshold) trackmode = true;
        else if (mse > 1.2 * signalthreshold) trackmode = false;

        if (!trackmode)
            st_osc.AdvanceFractionOfWave(-(1.0 - weighting) * st_angle_error * (st_gain_acq / 360.0));
        else
            st_osc.AdvanceFractionOfWave(-(1.0 - weighting) * st_angle_error * (st_gain_trk / 360.0));

        /* Timing NCO frequency adaptation (satdump M&M omega-gain
         * equivalent). JAERO had none: with a typical RTL-SDR crystal
         * ~25 ppm off, the true symbol clock in receiver samples is
         * ~0.01 Hz away from nominal, and phase nudges alone can't hold
         * it — the eye slowly closes and re-acquires in a limit cycle.
         * Learn the offset into the NCO frequency, clamped to +-0.1 Hz
         * of nominal so noise can't walk it away. */
        st_osc.IncreseFreqHz(-(1.0 - weighting) * st_angle_error * st_freq_gain);
        if (st_osc.GetFreqHz() < fb / 2.0 - 0.1) st_osc.SetFreq(fb / 2.0 - 0.1);
        if (st_osc.GetFreqHz() > fb / 2.0 + 0.1) st_osc.SetFreq(fb / 2.0 + 0.1);

        if (st_osc.IfHavePassedPoint(ee)) {
            double ct_xt = tanh(sig2.imag()) * sig2.real();
            double ct_xt_d = tanh(_pt_d.real()) * _pt_d.imag();
            double ct_ec = ct_xt_d - ct_xt;

            if (ct_ec > M_PI) ct_ec = M_PI;
            if (ct_ec < -M_PI) ct_ec = -M_PI;
            if (ct_ec > M_PI_2) ct_ec = M_PI_2;
            if (ct_ec < -M_PI_2) ct_ec = -M_PI_2;

            /* Designed 2nd-order PI carrier loop (see setSettings). alpha is
             * rad/rad; beta is rad/update^2, converted to Hz per update. */
            double ct_alpha = trackmode ? ct_alpha_trk : ct_alpha_acq;
            double ct_beta = trackmode ? ct_beta_trk : ct_beta_acq;
            mixer2.IncresePhaseDeg((180.0 / M_PI) * ct_alpha * ct_ec);
            mixer2.IncreseFreqHz((carrier_upd_rate / (2.0 * M_PI)) * ct_beta * ct_ec);

            /* Hard clamp mixer2 to the AFC capture range (satdump costas
             * freq_limit equivalent): the fine loop must not wander onto an
             * adjacent channel between coarse-estimator updates. */
            {
                double lo = mixer_center.GetFreqHz() - lockingbw / 2.0;
                double hi = mixer_center.GetFreqHz() + lockingbw / 2.0;
                if (mixer2.GetFreqHz() < lo) mixer2.SetFreq(lo);
                if (mixer2.GetFreqHz() > hi) mixer2.SetFreq(hi);
            }

            marg->UpdateSigned(ct_ec / 2.0);
            dt.update(pt_msk);
            pt_msk *= cpx_type(cos(marg->Val), sin(marg->Val));

            /* Ring-buffer the per-symbol constellation point for the web UI
             * scatter plot. Always on — cheap, used only when someone opens
             * the Spectrum tab. */
            if (!pointbuff.empty()) {
                pointbuff[pointbuff_ptr] = pt_msk;
                pointbuff_ptr = (pointbuff_ptr + 1) % (int)pointbuff.size();
            }

            double tda = (fabs(pt_msk.real() * 0.75) - 1.0);
            double tdb = (fabs(pt_msk.imag() * 0.75) - 1.0);
            mse = msema->Update((tda * tda) + (tdb * tdb));

            double imagin = diffdecode.UpdateSoft(pt_msk.imag());
            int ibit = qRoundI(imagin * 127.0 + 128.0);
            if (ibit > 255) ibit = 255;
            if (ibit < 0) ibit = 0;
            RxDataBits.push_back((short)ibit);

            double real = diffdecode.UpdateSoft(pt_msk.real());
            real = -real;
            ibit = qRoundI(real * 127.0 + 128.0);
            if (ibit > 255) ibit = 255;
            if (ibit < 0) ibit = 0;
            RxDataBits.push_back((short)ibit);

            if ((int)RxDataBits.size() >= 12) {
                if (soft_bits_cb)
                    soft_bits_cb(RxDataBits.data(), (int)RxDataBits.size(), soft_bits_user);
                RxDataBits.clear();
            }
        }

        mixer2.WTnextFrame();
        mixer_center.WTnextFrame();
        st_osc.WTnextFrame();
        ptr++;
    }
}

void MskDemodulator::feedAudio(const int16_t *samples, int num_samples, int sample_rate)
{
    (void)sample_rate;
    processAudio(samples, num_samples);
}

/* feedIQ: mix baseband IQ to int16 audio at freq_center via 125-tap
 * Hilbert USB demod (same as SDRReceiver/ZMQ path), then processAudio.
 * Measured ~1.5 dB better Eb/No than plain `re*cos - im*sin` on ch12. */
void MskDemodulator::feedIQ(const double *iq_interleaved, int num_samples)
{
    std::vector<int16_t> pcm(num_samples);
    feediq_usb.process(iq_interleaved, num_samples, Fs, freq_center, 5.0, pcm.data());
    processAudio(pcm.data(), num_samples);
}

