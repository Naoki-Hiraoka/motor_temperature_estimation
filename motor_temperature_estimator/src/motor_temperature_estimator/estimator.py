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
    eq = (sympy.Eq( Tcoil(t).diff(t), sympy.simplify( (param["Re"] * (tau/param["K"])** 2 - (Tcoil(t) - Thousing(t)) / param["R1"]) / param["Ccoil"]) ),
          sympy.Eq( Thousing(t).diff(t), sympy.simplify( ((Tcoil(t) - Thousing(t)) / param["R1"] - (Thousing(t) - Tair) / param["R2"]) / param["Chousing"] )))

    # sympy 1.1.1 do not support ics
    ans = sympy.dsolve(eq)
    return (ans[0].rhs, ans[0].lhs)


def calcSpecialSolutionOfTemperature(thermal_param, Tcoil_general, Thousing_general, Tair_current, Tcoil_current, Thousing_current):
    # thermal_param: MotorThermalParamConfig
    # Tcoil_general: General solution for temperature of coil [degree]
    # Thousing_general: General solution for temperature of housing [degree]
    # Tair_current: Current temperature of air [degree]
    # Tcoil_current: Current temperature of coil [degree]
    # Thousing_current: Current temperature of housing [degree]
    # return: (Tcoil, Thousing)

    sympy.var("C1 C2")
    dic = sympy.solve([sympy.Eq(Tcoil_general.subs([(Tair, Tair_current), (t,0)]),Tcoil_current),sympy.Eq(Thousing_general.subs([(Tair, Tair_current), (t,0)]),Thousing_current)],[C1,C2])
    return (Tcoil_general.subs([(Tair, Tair_current), ( C1, dic[C1] ), (C2, dic[C2] )]),
            Thousing_general.subs([(Tair, Tair_current), ( C1, dic[C1] ), (C2, dic[C2] )]))

def calcMaxTorqueAccurate(thermal_param, T_special, safe_time):
    return sympy.solve(sympy.Eq(T_special.subs( [ (t, safe_time) ]), self.thermal_param["Tlimit"]), tau)
