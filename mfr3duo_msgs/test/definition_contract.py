"""Verify the pinned upstream IDL and independently generated Python types."""
import hashlib
import json
from pathlib import Path
from mfr3duo_msgs.action import Move, Grasp
from mfr3duo_msgs.msg import GraspEpsilon
root = Path(__file__).resolve().parents[1]
upstream = json.loads((root / 'upstream.json').read_text())
for name, expected in upstream['sha256'].items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected, name
assert Move.Goal.get_fields_and_field_types() == {'width': 'double', 'speed': 'double'}
assert list(Grasp.Goal.get_fields_and_field_types()) == ['width', 'epsilon', 'speed', 'force']
for action in [Move, Grasp]:
    assert action.Result.get_fields_and_field_types() == {'success': 'boolean', 'error': 'string'}
    assert action.Feedback.get_fields_and_field_types() == {'current_width': 'double'}
assert GraspEpsilon().inner == GraspEpsilon().outer == 0.005
assert Grasp.Goal().epsilon.inner == Grasp.Goal().epsilon.outer == 0.005
print('PINNED_UPSTREAM_AND_GENERATED_TYPES_PASS', upstream['commit'])
