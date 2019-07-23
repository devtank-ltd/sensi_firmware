""" lbbc_firmware package"""
## @package lbbc_firmware
#
# LBBC firmware wrapper
#

VERSION = 1.0

##
# @brief Publically exportable modules
#
__all__ = [
    "binding",
]

##
# @brief Import all variables in these modules into lbbc_firmware namespace
#
# access as lbbc_firmware.<thing>, e.g. lbbc_firmware.io_board_py_t
#
from .binding import *
