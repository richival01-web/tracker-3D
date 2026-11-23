"""Smoke test: the package and its key third-party dependencies can be imported."""


def test_import_package():
    import nusc_tools  # noqa: F401


def test_import_devkit():
    from nuscenes.nuscenes import NuScenes  # noqa: F401
