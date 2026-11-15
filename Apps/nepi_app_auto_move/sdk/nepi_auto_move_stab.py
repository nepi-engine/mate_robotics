#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi engine (nepi_engine) repo
# (see https://github.com/nepi-engine/nepi_engine)
#
# License: NEPI Engine repo source-code and NEPI Images that use this source-code
# are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

import copy


########################
## Library Data

# Stabilization control definitions in the nepi_controls init-dict form. A node
# deep-copies this and hands it to AutoMoveIF, which builds the app's second
# ControlsIF from it at <app>/stab_controls.
#
# stab_method's 'options' and 'default' are left empty here and filled in from
# STAB_METHODS_DICT at the end of the method registry below. The registry is the
# single list of methods: adding a method there is what puts it in the selector,
# so this dict never has to be edited for a new method.
#
# stab_enabled carries 'param': False. That keeps it out of the ControlsIF's
# saved params, so it is never restored at boot, by a config restore or by a
# reset, and stabilization only ever starts from an operator's explicit Enable.
# Do not remove it, and do not add any other persistence for this control.
STAB_CONTROLS_DICT = {

    'stab_method': {
        'type': 'Selection', 'default': '', 'options': [],
        'display_name': 'Method',
        'description': 'Stabilization method to run while Enable is on.',
        'display_hidden': False},

    'stab_enabled': {
        'type': 'Toggle', 'default': False, 'param': False,
        'display_name': 'Enable',
        'description': 'Runs the selected stabilization method. Always starts '
                       'off, and is never restored from a saved config.',
        'display_hidden': False},

    }


#########################
# Stabilization Methods
#########################

##############################################################################
# EXTENSION POINT
#
# A stabilization method is one function plus one entry in STAB_METHODS_DICT
# below. The selector's options follow the registry automatically, in registry
# order, and the first entry is the default.
#
# WHAT TO CHANGE
#   Add a function with the signature below and register it under the display
#   name the operator should see. Replace the body of stab_optical_flow() when
#   the real optical flow method lands.
#
# WHAT NOT TO CHANGE
#   The signature:
#
#       method(stab_dict, robot_dict, controls_dict) -> result dict
#
#   AutoMoveIF calls run_stab_method() with exactly these arguments once per
#   stabilization cycle, and run_stab_method() hands them straight to the
#   registered function.
#
#   stab_dict carries the loop's own bookkeeping:
#       'method'       (str)   the method's registered display name
#       'cycle_count'  (int)   cycles since this run started, 0 on the first
#       'run_time_s'   (float) seconds since stabilization was turned on or the
#                              method last changed
#       'dt_s'         (float) seconds since the previous cycle, 0.0 on the first
#   A run restarts, and every counter with it, when Enable goes on and when the
#   method changes while on. A method that keeps state between cycles should
#   reset it when cycle_count is 0.
#
#   robot_dict is AutoMoveIF.getRobotDict(): namespace, connected, ready and
#   status_dict, as the RBX connect reports them. controls_dict is a copy of the
#   live stab_controls dict.
#
#   Do not import ROS, nepi_sdk transport, or any interface class here. This
#   module is a pure function of its arguments so a test can call it with
#   fixtures and any other node can call it with the same arguments.
#
#   Do not block. A method runs on the stabilization timer thread, once per
#   0.1 s cycle, and the controls callback waits on that same lock while it
#   runs. A method that needs to iterate should do one step per call and carry
#   on at the next cycle.
#
# THE RESULT
#   Return a dict. This version reads only 'msg' from it: a str that the app
#   copies into stab_msg on its status message, so it is worth writing for an
#   operator. A method that raises puts the loop in the error state with the
#   exception as the message; Enable stays on and the loop retries next cycle.
#
#   In this version nothing commands the robot. The first real method -- the
#   optical flow work -- defines whatever command keys it needs in this result,
#   with their units and frame, and the AutoMoveIF change that acts on them.
#   Until then any key other than 'msg' is ignored.
##############################################################################

def stab_optical_flow(stab_dict, robot_dict, controls_dict):
    """Placeholder optical flow stabilization method.

    Sends nothing and reads nothing but the run time. It exists so the
    selector, the enable toggle, the loop and the status readout can be
    exercised end to end before the real optical flow method is written.

    Args:
        stab_dict (dict): Loop bookkeeping, see the EXTENSION POINT above.
        robot_dict (dict): AutoMoveIF.getRobotDict() output. Unused.
        controls_dict (dict): Copy of the live stab_controls dict. Unused.

    Returns:
        dict: {'msg': str} saying this is a placeholder, that no commands are
            sent, and how long it has been running.
    """
    run_time_s = 0.0
    try:
        run_time_s = float(stab_dict.get('run_time_s', 0.0))
    except (TypeError, ValueError, AttributeError):
        run_time_s = 0.0
    return {
        'msg': ('Optical Flow placeholder, no commands sent. Running ' +
                str(round(run_time_s, 1)) + ' s'),
    }


# Method registry: display name -> function. Order is the selector's option
# order (dicts keep insertion order), and the first entry is its default.
STAB_METHODS_DICT = {
    'Optical Flow': stab_optical_flow,
}

# Fill the selector from the registry, so a new method needs no edit to
# STAB_CONTROLS_DICT above.
STAB_CONTROLS_DICT['stab_method']['options'] = list(STAB_METHODS_DICT.keys())
STAB_CONTROLS_DICT['stab_method']['default'] = (list(STAB_METHODS_DICT.keys())[0]
                                                if len(STAB_METHODS_DICT) > 0 else '')


def run_stab_method(method_name, stab_dict, robot_dict, controls_dict):
    """Run one cycle of a registered stabilization method.

    Dispatches to the function registered under method_name and returns its
    result unchanged. An exception the method raises is NOT caught here; the
    caller owns the error state.

    Args:
        method_name (str): Display name of the method, as the stab_method
            selector reports it.
        stab_dict (dict): Loop bookkeeping: method, cycle_count, run_time_s,
            dt_s.
        robot_dict (dict): AutoMoveIF.getRobotDict() output.
        controls_dict (dict): Copy of the live stab_controls dict.

    Returns:
        dict: The method's result, which carries at least 'msg' (str). For a
            name that is not registered, a result whose 'msg' names it,
            instead of raising.
    """
    method_function = STAB_METHODS_DICT.get(method_name, None)
    if method_function is None:
        return {
            'msg': ('Unknown stabilization method: ' + str(method_name) +
                    '. No commands sent.'),
        }
    return method_function(stab_dict, robot_dict, controls_dict)


def get_stab_method_names():
    """Return the registered stabilization method names, in registry order.

    Returns:
        list: The display names, which are also the stab_method selector's
            options. The first is the selector's default.
    """
    return list(STAB_METHODS_DICT.keys())


def get_stab_controls_dict():
    """Return a copy of this module's stabilization control definition dictionary.

    Returns:
        dict: The nepi_controls init-dict form of STAB_CONTROLS_DICT, safe for
            a caller to hand to a ControlsIF and modify.
    """
    return copy.deepcopy(STAB_CONTROLS_DICT)
