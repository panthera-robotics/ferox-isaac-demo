"""Isaac Lab configuration for the twin assets.

Importable WITHOUT Isaac Lab installed. Everything in here is plain data until you
call a `build_*` function, which lazy-imports `isaaclab` at that point. That is what
lets the tests run on this box, in CI, and anywhere else that has no Isaac Lab.
"""
