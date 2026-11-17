import csv
from datetime import datetime
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import csv_features
from protocol import Parser, simulated_packet, CSV_COLUMNS


class RecorderTests(unittest.TestCase):
    def test_incremental_all_columns_and_exclusive_file(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(csv_features, 'ROOT', Path(directory)):
            recorder = csv_features.CsvRecorder()
            path = recorder.start()
            self.assertEqual(path.parent, Path(directory)/'data')
            datetime.strptime(path.stem, '%Y%m%d_%H%M%S_%f')
            self.assertEqual(recorder.start(), path)
            frames = Parser().feed(simulated_packet(20)+simulated_packet(21))
            recorder.write(10, frames[0]); recorder.write(11, frames[1])
            recorder.flush()
            with path.open(encoding='utf-8-sig') as stream:
                rows = list(csv.reader(stream))
            self.assertEqual(rows[0], list(CSV_COLUMNS))
            self.assertEqual(len(rows), 3)
            self.assertAlmostEqual(float(rows[1][1]), frames[0].time_s)
            recorder.stop()
            recorder.write(12, frames[1])
            self.assertEqual(recorder.rows, 2)
            with patch.object(csv_features, 'datetime') as clock:
                clock.now.return_value.strftime.return_value = path.stem
                with self.assertRaises(FileExistsError):
                    recorder.start()
            self.assertEqual(path.read_text(encoding='utf-8-sig').count('\n'), 3)


if __name__ == '__main__':
    unittest.main()
