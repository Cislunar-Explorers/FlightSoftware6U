from math import sqrt

MU_EARTH = 3.986004418e5  # km^3s^-2

R_EARTH = 6378  # km
LEO_ALTITUDE = 400  # km, approx
GEO_ALTITUDE = 35786  # km
MOON_ALTITUDE = 400000  # km


def orbit_raise_dv(periapsis, old_apoapsis, new_apoapsis, mu=MU_EARTH):
    # calculates the delta-V required to do an apoapsis raise
    # units are in km
    a_initial = (periapsis + old_apoapsis) / 2
    a_final = (periapsis + new_apoapsis) / 2
    delta_v = sqrt(2 * mu / periapsis - mu / a_final) - sqrt(2 * mu / periapsis - mu / a_initial)
    return delta_v


if __name__ == "__main__":
    print(f"GEO: {orbit_raise_dv(R_EARTH + GEO_ALTITUDE, R_EARTH + GEO_ALTITUDE, R_EARTH + MOON_ALTITUDE):.3f} km/s")
    print(f"GTO: {orbit_raise_dv(R_EARTH + LEO_ALTITUDE, R_EARTH + GEO_ALTITUDE, R_EARTH + MOON_ALTITUDE):.3f} km/s")
