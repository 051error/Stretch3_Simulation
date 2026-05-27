"""Resolve paths relative to the repository root."""

from pathlib import Path


def get_repo_root() -> Path:
    """Repository root (parent of ``src/``)."""
    return Path(__file__).resolve().parents[2]


def get_path(*path_parts: str) -> Path:
    """Absolute path under the repository root."""
    return get_repo_root().joinpath(*path_parts)


def get_xml_path(filename: str = "table_world.xml") -> Path:
    """Path to a MuJoCo model in ``models/``."""
    return get_path("models", filename)


def get_mesh_path(filename: str) -> Path:
    """Path to a mesh in ``meshes/``."""
    return get_path("meshes", filename)


def get_texture_path(filename: str) -> Path:
    """Path to a texture in ``textures/``."""
    return get_path("textures", filename)


def get_asset_path(filename: str) -> Path:
    """Path to a robot asset in ``assets/``."""
    return get_path("assets", filename)


def get_config_path(filename: str) -> Path:
    """Path to a config file in ``config/``."""
    return get_path("config", filename)
