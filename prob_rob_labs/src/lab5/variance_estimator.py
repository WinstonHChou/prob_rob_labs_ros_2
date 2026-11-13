def measurement_variances(prior_distance_m: float) -> tuple[float, float]:
    if prior_distance_m < 0:
        raise ValueError("Distance cannot be negative")

    if prior_distance_m < 1.0:
        sigma_distance = 7.1
        sigma_bearing = 0.07
    else:
        sigma_distance = max(0.01, 0.013 * prior_distance_m)
        sigma_bearing = 0.003

    return sigma_distance**2, sigma_bearing**2


if __name__ == '__main__':
    print("Demo values for different distances")
    for d in [0.25, 1.5, 3.0, 5.0, 7.5, 10.0]:
        var_d, var_b = measurement_variances(d)
        print(f'{d:4.2f} m: var(distance)={var_d:.6f} m², var(bearing)={var_b:.8f} rad²')