#!/usr/bin/env python

from motor_temperature_estimator_msgs.cfg import MotorThermalParamConfig
import sympy

def estimateTemperature(Tcoil, Thousing, Tair, thermal_param, tau, dt):
    # Tcoil: Current temperature of coil [degree]
    # Thousing: Current temperature of housing [degree]
    # Tair: Current temperature of air [degree]
    # thermal_param: MotorThermalParamConfig
    # tau [Nm]
    # dt [s]
    # return: (Tcoil, Thousing)

    # Qin[W] -> coil -> Qmid[W] -> housing -> Qout[W]
    Qin = thermal_param["Re"] * pow(tau/thermal_param["K"], 2)
    Qmid = (Tcoil - Thousing) / thermal_param["R1"];
    Qout = (Thousing - Tair) / thermal_param["R2"];

    return (Tcoil + (Qin - Qmid) / thermal_param["Ccoil"] * dt,
            Thousing + (Qmid - Qout) / thermal_param["Chousing"] * dt)

def calcGeneralSolutionOfTemperature(thermal_param):
    # thermal_param: MotorThermalParamConfig
    # return: (Tcoil, Thousing)

    Tcoil, Thousing = sympy.symbols("Tcoil Thousing", cls = sympy.Function)
    t = sympy.Symbol("t")
    sympy.var("tau Tair")
    eq = (sympy.Eq( Tcoil(t).diff(t), sympy.simplify( (thermal_param["Re"] * (tau/thermal_param["K"])** 2 - (Tcoil(t) - Thousing(t)) / thermal_param["R1"]) / thermal_param["Ccoil"]) ),
          sympy.Eq( Thousing(t).diff(t), sympy.simplify( ((Tcoil(t) - Thousing(t)) / thermal_param["R1"] - (Thousing(t) - Tair) / thermal_param["R2"]) / thermal_param["Chousing"] )))

    # sympy 1.1.1 do not support ics
    ans = sympy.dsolve(eq)
    sympy.var("Tcoil0 Thousing0")
    sympy.var("C1 C2")
    dic = sympy.solve([sympy.Eq(ans[0].rhs.subs([(t,0)]),Tcoil0),sympy.Eq(ans[1].rhs.subs([(t,0)]),Thousing0)],[C1,C2])
    return (ans[0].rhs.subs([( C1, dic[C1] ), (C2, dic[C2] )]),
            ans[1].rhs.subs([( C1, dic[C1] ), (C2, dic[C2] )]))

def calcSpecialSolutionOfTemperature(thermal_param, Tcoil_general, Thousing_general, Tair_current, Tcoil_current, Thousing_current):
    # thermal_param: MotorThermalParamConfig
    # Tcoil_general: General solution for temperature of coil [degree]
    # Thousing_general: General solution for temperature of housing [degree]
    # Tair_current: Current temperature of air [degree]
    # Tcoil_current: Current temperature of coil [degree]
    # Thousing_current: Current temperature of housing [degree]
    # return: (Tcoil, Thousing)

    return (Tcoil_general.subs([(Tair, Tair_current), (Tcoil0, Tcoil_current ),(Thousing0, Thousing_current )]),
            Thousing_general.subs([(Tair, Tair_current), (Tcoil0, Tcoil_current ),(Thousing0, Thousing_current )]))

def calcMaxTorqueAccurate(thermal_param, T_special, safe_time):
    t = sympy.Symbol("t")
    taus = sympy.solve(sympy.Eq(T_special.subs( [ (t, safe_time) ]), thermal_param["Tlimit"]), tau, rational=False) # rational=False: for speed up
    if not taus:
        return 0.0
    valid_taus = []
    for effort in taus:
        try:
            if effort >= 0.0:
                valid_taus.append(effort)
        except:
            pass
    if len(valid_taus) == 0:
        return 1e10
    return min(valid_taus)


def calcRemainingTimeAccurate(thermal_param, T_special, tau_current):
    # unstale. should not be used.

    t = sympy.Symbol("t")
    if T_special.subs([ (t, 0.0), (tau, 0.0) ]) >= thermal_param["Tlimit"]:
        return 0.0

    try:
        times = sympy.solve(sympy.Eq(T_special.subs( [ (tau, tau_current) ]), thermal_param["Tlimit"]), t)
    except:
        return 0.0
    if not times:
        return 0.0
    valid_times = []
    for time in times:
        try:
            if time >= 0.0:
                valid_times.append(time)
        except:
            pass
    if len(valid_times) == 0:
        return 1e10
    return min(valid_times)

def calcRemainingTimeFast(thermal_param, T_special, tau_current):
    t = sympy.Symbol("t")
    if T_special.subs([ (t, 0.0), (tau, 0.0) ]) >= thermal_param["Tlimit"]:
        return 0.0

    T_special = T_special.subs([(tau, tau_current)])
    for time in range(0, 300, 10):
        if T_special.subs([(t, time)]) > thermal_param["Tlimit"]:
            return time
    return 300
