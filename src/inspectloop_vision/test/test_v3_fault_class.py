"""V3 (Astra V2 review, F7): typed observation outcomes - system faults are not occlusion."""
import pytest

from inspectloop_vision.gauge_reader_node import fault_class_for


@pytest.mark.parametrize('has_value,flags,expected', [
    (True, [], 'NONE'),
    (True, ['TAG_INTERMITTENT'], 'NONE'),
    (False, ['NO_IMAGE'], 'CAMERA_UNAVAILABLE'),
    (False, ['INVALID_FRAMES'], 'IMAGE_INVALID'),
    (False, ['FROZEN_FRAMES', 'TAG_NOT_VISIBLE'], 'IMAGE_INVALID'),
    (False, ['CLOCK_RESET'], 'IMAGE_INVALID'),
    (False, ['WRONG_VIEWPOINT'], 'IMAGE_INVALID'),
    (False, ['ROBOT_MOVING'], 'IMAGE_INVALID'),
    (False, ['PROCESSING_ERROR'], 'PROCESSING_ERROR'),
    (False, ['OCCLUDED'], 'INSUFFICIENT_EVIDENCE'),
    (False, ['AMBIGUOUS_NEEDLE', 'OBLIQUE_VIEW'], 'INSUFFICIENT_EVIDENCE'),
    (False, ['TAG_NOT_VISIBLE'], 'INSUFFICIENT_EVIDENCE'),
])
def test_fault_class(has_value, flags, expected):
    assert fault_class_for(has_value, flags) == expected


def test_processing_error_dominates_and_is_never_no_image():
    assert fault_class_for(False, ['PROCESSING_ERROR', 'OCCLUDED']) == 'PROCESSING_ERROR'


def test_exception_path_reports_processing_error_not_no_image():
    """An exception inside the reader must be typed PROCESSING_ERROR (V2 reported NO_IMAGE)."""
    from types import SimpleNamespace

    from inspectloop_interfaces.srv import ReadGauge
    from inspectloop_vision.gauge_reader_node import GaugeReaderNode

    def boom(_req, _resp):
        raise ValueError('synthetic reader bug')
    logs = []
    fake = SimpleNamespace(_read=boom, get_logger=lambda: SimpleNamespace(error=logs.append))
    req = ReadGauge.Request()
    req.gauge_id, req.observation_id = 'PG-11', 'm/PG-11/V0/1'
    resp = GaugeReaderNode._on_read(fake, req, ReadGauge.Response())
    assert resp.success is False
    assert list(resp.reading.quality_flags) == ['PROCESSING_ERROR']
    assert resp.reading.fault_class == 'PROCESSING_ERROR'
    assert 'NO_IMAGE' not in resp.reading.quality_flags
    assert resp.reading.observation_id == 'm/PG-11/V0/1'
    assert logs
