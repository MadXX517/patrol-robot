import unittest

from car_report.security_events import (
    ConfirmFrameTracker,
    parse_target_classes,
    security_event_metadata,
)


class SecurityEventsTest(unittest.TestCase):
    def test_person_maps_to_intrusion_event(self):
        metadata = security_event_metadata('person')

        self.assertEqual(metadata['event_type'], 'intrusion_detected')
        self.assertEqual(metadata['event_name'], '核心禁区人员闯入')
        self.assertEqual(metadata['priority'], 'highest')
        self.assertEqual(metadata['risk_level'], 'high')

    def test_bag_classes_map_to_unattended_object_event(self):
        for class_name in ('backpack', 'suitcase', 'handbag'):
            metadata = security_event_metadata(class_name)

            self.assertEqual(metadata['event_type'], 'unattended_object')
            self.assertEqual(metadata['event_name'], '设备保障区遗留物')
            self.assertEqual(metadata['priority'], 'high')
            self.assertEqual(metadata['risk_level'], 'medium')

    def test_confirm_frame_tracker_requires_three_hits(self):
        tracker = ConfirmFrameTracker(confirm_frames=3)

        self.assertEqual(tracker.update({'person'}), set())
        self.assertEqual(tracker.update({'person'}), set())
        self.assertEqual(tracker.update({'person'}), {'person'})

    def test_confirm_frame_tracker_resets_when_target_disappears(self):
        tracker = ConfirmFrameTracker(confirm_frames=3)

        self.assertEqual(tracker.update({'person'}), set())
        self.assertEqual(tracker.update(set()), set())
        self.assertEqual(tracker.update({'person'}), set())
        self.assertEqual(tracker.update({'person'}), set())
        self.assertEqual(tracker.update({'person'}), {'person'})

    def test_parse_target_classes_matches_competition_classes(self):
        targets = parse_target_classes('person,backpack,suitcase,handbag')

        self.assertEqual(targets, {'person', 'backpack', 'suitcase', 'handbag'})


if __name__ == '__main__':
    unittest.main()
