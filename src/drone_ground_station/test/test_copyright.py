from ament_copyright.main import main
import pytest


@pytest.mark.skip(
    reason="Generated Python sources do not yet carry copyright headers."
)
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found errors"
