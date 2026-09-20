"""RH56E2 installed-hand measurement preparation (CPU only until hardware work is authorized).

Everything here is preparation: a register map transcribed from the manufacturer manual with page references, transports
that refuse to open unless hardware work is explicitly authorized, loggers that write a fixed record format, blank
measurement sheets, a native-to-radian conversion template that refuses to emit a map from missing values, and a comparison
tool that reports NOT_MEASURED rather than a number. No value in this package is an installed measurement.
"""
