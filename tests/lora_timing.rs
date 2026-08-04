#![allow(dead_code)]

#[path = "../src/lora_timing.rs"]
mod lora_timing;

use lora_timing::{
    AUX_DONE_INTERVAL, AUX_LOW_DURATION, IDLE_GAP, PAYLOAD_PREPARE, REQUEST_INTERVAL,
    RequestSource, SAMPLE_INTERVALS, TimingCollector, TxTimingTrace, UART_WRITE,
    WRITE_START_INTERVAL,
};

fn complete_trace(t0: u64, source: RequestSource, low_observed: bool) -> TxTimingTrace {
    let mut trace = TxTimingTrace::new(t0, source);
    trace.t3 = Some(t0 + 1);
    trace.t4 = Some(t0 + 3);
    trace.t1 = Some(t0 + 5);
    trace.t2 = Some(t0 + 7);
    trace.t5 = Some(t0 + 10);
    trace.t6 = Some(t0 + 14);
    trace.t7 = Some(t0 + 15);
    trace.t8 = low_observed.then_some(t0 + 17);
    trace.t9 = Some(t0 + 20);
    trace.aux_low_not_observed = !low_observed;
    trace
}

#[test]
fn first_trace_only_establishes_interval_baselines() {
    let mut collector = TimingCollector::new();
    assert!(
        collector
            .record(complete_trace(100, RequestSource::Periodic, true))
            .is_none()
    );
}

#[test]
fn reports_ten_intervals_with_correct_stats_and_counters() {
    let mut collector = TimingCollector::new();
    assert!(
        collector
            .record(complete_trace(0, RequestSource::Periodic, true))
            .is_none()
    );

    let mut report = None;
    let mut t0 = 0u64;
    for index in 1..=SAMPLE_INTERVALS {
        t0 += 100 + u64::from(index);
        let source = if index == 3 {
            RequestSource::TopTrigger
        } else {
            RequestSource::Periodic
        };
        let trace = complete_trace(t0, source, index != 4);
        report = collector.record(trace);
        assert_eq!(report.is_some(), index == SAMPLE_INTERVALS);
    }

    let report = report.unwrap();
    assert_eq!(report.metrics[REQUEST_INTERVAL].count, SAMPLE_INTERVALS);
    assert_eq!(report.metrics[REQUEST_INTERVAL].min, 101);
    assert_eq!(report.metrics[REQUEST_INTERVAL].max, 110);
    assert_eq!(report.metrics[REQUEST_INTERVAL].average, 105);
    assert_eq!(report.metrics[PAYLOAD_PREPARE].average, 2);
    assert_eq!(report.metrics[UART_WRITE].average, 4);
    assert_eq!(report.metrics[AUX_LOW_DURATION].count, SAMPLE_INTERVALS - 1);
    assert_eq!(report.metrics[WRITE_START_INTERVAL].count, SAMPLE_INTERVALS);
    assert_eq!(report.metrics[AUX_DONE_INTERVAL].count, SAMPLE_INTERVALS);
    assert_eq!(report.metrics[IDLE_GAP].count, SAMPLE_INTERVALS);
    assert_eq!(report.aux_low_not_observed, 1);
    assert_eq!(report.periodic_requests, 9);
    assert_eq!(report.top_trigger_requests, 1);
    assert_eq!(report.idle_gap_invalid, 0);
}

#[test]
fn report_reset_keeps_previous_timestamps_for_next_window() {
    let mut collector = TimingCollector::new();
    assert!(
        collector
            .record(complete_trace(0, RequestSource::Periodic, true))
            .is_none()
    );
    for index in 1..=SAMPLE_INTERVALS {
        let report = collector.record(complete_trace(
            u64::from(index) * 100,
            RequestSource::Periodic,
            true,
        ));
        assert_eq!(report.is_some(), index == SAMPLE_INTERVALS);
    }

    let mut second_report = None;
    for index in 11u64..=20 {
        second_report =
            collector.record(complete_trace(index * 100, RequestSource::Periodic, true));
    }
    let second_report = second_report.unwrap();
    assert_eq!(
        second_report.metrics[REQUEST_INTERVAL].count,
        SAMPLE_INTERVALS
    );
    assert_eq!(second_report.metrics[REQUEST_INTERVAL].min, 100);
    assert_eq!(second_report.metrics[REQUEST_INTERVAL].max, 100);
    assert_eq!(second_report.metrics[REQUEST_INTERVAL].average, 100);
}

#[test]
fn negative_idle_gap_is_not_recorded() {
    let mut collector = TimingCollector::new();
    let mut first = complete_trace(100, RequestSource::Periodic, true);
    first.t9 = Some(300);
    assert!(collector.record(first).is_none());

    let report = (1..=SAMPLE_INTERVALS)
        .find_map(|index| {
            collector.record(complete_trace(
                110 + u64::from(index) * 100,
                RequestSource::Periodic,
                true,
            ))
        })
        .unwrap();
    assert_eq!(report.metrics[IDLE_GAP].count, SAMPLE_INTERVALS - 1);
    assert_eq!(report.idle_gap_invalid, 1);
}
