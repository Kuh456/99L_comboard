pub(crate) const SAMPLE_INTERVALS: u32 = 10;

pub(crate) const REQUEST_INTERVAL: usize = 0;
pub(crate) const PRE_AUX_WAIT: usize = 1;
pub(crate) const INITIAL_AUX_WAIT: usize = 2;
pub(crate) const RX_GUARD_WAIT: usize = 3;
pub(crate) const POST_GUARD_AUX_WAIT: usize = 4;
pub(crate) const PAYLOAD_PREPARE: usize = 5;
pub(crate) const UART_WRITE: usize = 6;
pub(crate) const UART_FLUSH: usize = 7;
pub(crate) const AUX_LOW_DELAY: usize = 8;
pub(crate) const AUX_LOW_DURATION: usize = 9;
pub(crate) const POST_UART_TO_DONE: usize = 10;
pub(crate) const TX_TOTAL: usize = 11;
pub(crate) const WRITE_START_INTERVAL: usize = 12;
pub(crate) const AUX_DONE_INTERVAL: usize = 13;
pub(crate) const IDLE_GAP: usize = 14;
pub(crate) const METRIC_COUNT: usize = 15;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[allow(dead_code)]
pub(crate) enum RequestSource {
    Periodic,
    TopTrigger,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct TxTimingTrace {
    pub t0: u64,
    pub t1: Option<u64>,
    pub t2: Option<u64>,
    pub t3: Option<u64>,
    pub t4: Option<u64>,
    pub t5: Option<u64>,
    pub t6: Option<u64>,
    pub t7: Option<u64>,
    pub t8: Option<u64>,
    pub t9: Option<u64>,
    pub aux_low_not_observed: bool,
    pub source: RequestSource,
}

impl TxTimingTrace {
    pub const fn new(t0: u64, source: RequestSource) -> Self {
        Self {
            t0,
            t1: None,
            t2: None,
            t3: None,
            t4: None,
            t5: None,
            t6: None,
            t7: None,
            t8: None,
            t9: None,
            aux_low_not_observed: false,
            source,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct MetricSummary {
    pub count: u32,
    pub min: u64,
    pub max: u64,
    pub average: u64,
}

#[derive(Clone, Copy)]
struct MetricStats {
    count: u32,
    min: u64,
    max: u64,
    total: u64,
}

impl MetricStats {
    const fn new() -> Self {
        Self {
            count: 0,
            min: u64::MAX,
            max: 0,
            total: 0,
        }
    }

    fn record(&mut self, value: u64) {
        self.count = self.count.saturating_add(1);
        self.min = self.min.min(value);
        self.max = self.max.max(value);
        self.total = self.total.saturating_add(value);
    }

    fn summary(self) -> MetricSummary {
        if self.count == 0 {
            return MetricSummary {
                count: 0,
                min: 0,
                max: 0,
                average: 0,
            };
        }
        MetricSummary {
            count: self.count,
            min: self.min,
            max: self.max,
            average: self.total / u64::from(self.count),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct TimingReport {
    pub metrics: [MetricSummary; METRIC_COUNT],
    pub aux_low_not_observed: u32,
    pub periodic_requests: u32,
    pub top_trigger_requests: u32,
    pub idle_gap_invalid: u32,
}

pub(crate) struct TimingCollector {
    metrics: [MetricStats; METRIC_COUNT],
    previous_t0: Option<u64>,
    previous_t5: Option<u64>,
    previous_t9: Option<u64>,
    aux_low_not_observed: u32,
    periodic_requests: u32,
    top_trigger_requests: u32,
    idle_gap_invalid: u32,
}

impl TimingCollector {
    pub const fn new() -> Self {
        Self {
            metrics: [MetricStats::new(); METRIC_COUNT],
            previous_t0: None,
            previous_t5: None,
            previous_t9: None,
            aux_low_not_observed: 0,
            periodic_requests: 0,
            top_trigger_requests: 0,
            idle_gap_invalid: 0,
        }
    }

    pub fn record(&mut self, trace: TxTimingTrace) -> Option<TimingReport> {
        let Some(previous_t0) = self.previous_t0.replace(trace.t0) else {
            self.previous_t5 = trace.t5;
            self.previous_t9 = trace.t9;
            return None;
        };

        self.record_delta(REQUEST_INTERVAL, Some(trace.t0), Some(previous_t0));
        self.record_delta(PRE_AUX_WAIT, trace.t1, Some(trace.t0));
        self.record_delta(INITIAL_AUX_WAIT, trace.t1, trace.t4);
        self.record_delta(RX_GUARD_WAIT, trace.t2, trace.t1);
        self.record_delta(POST_GUARD_AUX_WAIT, trace.t5, trace.t2);
        self.record_delta(PAYLOAD_PREPARE, trace.t4, trace.t3);
        self.record_delta(UART_WRITE, trace.t6, trace.t5);
        self.record_delta(UART_FLUSH, trace.t7, trace.t6);
        self.record_delta(AUX_LOW_DELAY, trace.t8, trace.t7);
        self.record_delta(AUX_LOW_DURATION, trace.t9, trace.t8);
        self.record_delta(POST_UART_TO_DONE, trace.t9, trace.t7);
        self.record_delta(TX_TOTAL, trace.t9, Some(trace.t0));

        if let Some(t5) = trace.t5 {
            self.record_delta(WRITE_START_INTERVAL, Some(t5), self.previous_t5);
            self.previous_t5 = Some(t5);
            if let Some(previous_t9) = self.previous_t9 {
                if let Some(idle_gap) = t5.checked_sub(previous_t9) {
                    self.metrics[IDLE_GAP].record(idle_gap);
                } else {
                    self.idle_gap_invalid = self.idle_gap_invalid.saturating_add(1);
                }
            }
        }
        if let Some(t9) = trace.t9 {
            self.record_delta(AUX_DONE_INTERVAL, Some(t9), self.previous_t9);
            self.previous_t9 = Some(t9);
        }

        if trace.aux_low_not_observed {
            self.aux_low_not_observed = self.aux_low_not_observed.saturating_add(1);
        }
        match trace.source {
            RequestSource::Periodic => {
                self.periodic_requests = self.periodic_requests.saturating_add(1)
            }
            RequestSource::TopTrigger => {
                self.top_trigger_requests = self.top_trigger_requests.saturating_add(1)
            }
        }

        if self.metrics[REQUEST_INTERVAL].count < SAMPLE_INTERVALS {
            return None;
        }

        let report = TimingReport {
            metrics: self.metrics.map(MetricStats::summary),
            aux_low_not_observed: self.aux_low_not_observed,
            periodic_requests: self.periodic_requests,
            top_trigger_requests: self.top_trigger_requests,
            idle_gap_invalid: self.idle_gap_invalid,
        };
        self.reset_window();
        Some(report)
    }

    fn record_delta(&mut self, metric: usize, later: Option<u64>, earlier: Option<u64>) {
        if let (Some(later), Some(earlier)) = (later, earlier)
            && let Some(delta) = later.checked_sub(earlier)
        {
            self.metrics[metric].record(delta);
        }
    }

    fn reset_window(&mut self) {
        self.metrics = [MetricStats::new(); METRIC_COUNT];
        self.aux_low_not_observed = 0;
        self.periodic_requests = 0;
        self.top_trigger_requests = 0;
        self.idle_gap_invalid = 0;
    }
}
