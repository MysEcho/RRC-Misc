import csv


def read_csv_to_lists(file_path):
    """
    Reads x,y and z components of velocity and time difference from csv file. Refere Acceleration_test1.py for more clarity.
    """
    x_velocity = []
    y_velocity = []
    z_velocity = []
    time_elapsed = []
    with open(file_path, "r") as file:
        csv_reader = csv.reader(file)
        next(csv_reader)
        line_count = 0
        for row in csv_reader:
            line_count += 1
            if line_count > 2:
                if len(row) == 4:
                    x_velocity.append(float(row[0]))
                    y_velocity.append(float(row[1]))
                    z_velocity.append(float(row[2]))
                    time_elapsed.append(float(row[3]))
                else:
                    print(
                        f"Skipping row {csv_reader.line_num}: Expected 4 columns, found {len(row)}"
                    )
    return x_velocity, y_velocity, z_velocity, time_elapsed


def calculate_distance(x_velocity, y_velocity, z_velocity, time_elapsed):
    total_distance = 0.0
    for i in range(1, len(time_elapsed)):
        displacement_x = (
            (x_velocity[i - 1] + x_velocity[i])
            / 2
            * (time_elapsed[i] - time_elapsed[i - 1])
        )
        displacement_y = (
            (y_velocity[i - 1] + y_velocity[i])
            / 2
            * (time_elapsed[i] - time_elapsed[i - 1])
        )
        displacement_z = (
            (z_velocity[i - 1] + z_velocity[i])
            / 2
            * (time_elapsed[i] - time_elapsed[i - 1])
        )

        total_distance += (
            displacement_x**2 + displacement_y**2 + displacement_z**2
        ) ** 0.5

    return total_distance


csv_file_path = "data.csv"

x_velocity, y_velocity, z_velocity, time_elapsed = read_csv_to_lists(csv_file_path)

distance_covered = calculate_distance(x_velocity, y_velocity, z_velocity, time_elapsed)

print("Total distance covered:", distance_covered)
