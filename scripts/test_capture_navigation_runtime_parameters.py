from types import SimpleNamespace
import unittest

from capture_navigation_runtime_parameters import parameter_value


class ParameterEvidenceTest(unittest.TestCase):
    def test_false_zero_arrays_and_missing_are_not_conflated(self):
        self.assertIs(parameter_value(SimpleNamespace(type=1, bool_value=False)), False)
        self.assertEqual(parameter_value(SimpleNamespace(type=2, integer_value=0)), 0)
        self.assertIsNone(parameter_value(SimpleNamespace(type=0)))
        self.assertEqual(parameter_value(SimpleNamespace(type=8, double_array_value=(0.242, 0.007))),
                         [0.242, 0.007])
        with self.assertRaises(ValueError):
            parameter_value(SimpleNamespace(type=10))


if __name__ == '__main__':
    unittest.main()
