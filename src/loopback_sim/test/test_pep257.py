
from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257() -> None:
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
