import unittest

from car_yolo.detection_filter import parse_target_classes, should_keep_class


class DetectionFilterTest(unittest.TestCase):
    def test_empty_target_classes_keep_all(self):
        self.assertEqual(parse_target_classes(''), set())
        self.assertTrue(should_keep_class('chair', set()))

    def test_target_classes_keep_only_requested_classes(self):
        targets = parse_target_classes('person, backpack, suitcase, handbag')

        self.assertTrue(should_keep_class('person', targets))
        self.assertTrue(should_keep_class('backpack', targets))
        self.assertTrue(should_keep_class('suitcase', targets))
        self.assertTrue(should_keep_class('handbag', targets))
        self.assertFalse(should_keep_class('chair', targets))
        self.assertFalse(should_keep_class('bottle', targets))
        self.assertFalse(should_keep_class('laptop', targets))


if __name__ == '__main__':
    unittest.main()
