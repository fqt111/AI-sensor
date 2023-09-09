def adc_to_acceleration(adc_value, range=2):
    adc_resolution = 2**16
    voltage_from_adc = adc_value / adc_resolution
    acceleration = voltage_from_adc * range
    return acceleration


# Example values (please replace with actual values from your system)
range = 16  # for example, 0.3V/g

acceleration = adc_to_acceleration(adc_value, range)
print(acceleration)  # This will print the acceleration in 'g' units
