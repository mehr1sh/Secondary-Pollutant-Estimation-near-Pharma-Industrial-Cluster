import csv
import os

## --- CONFIGURATION --- ##
SOURCE_FILENAME = "activity_log_variance.csv"
ORIGINAL_INTERVAL_SECONDS = 5
## --- END OF CONFIGURATION --- ##


def resample_csv():
    """
    Prompts the user for a new time interval and creates a new CSV file
    by resampling the source data file.
    """
    # --- 1. Check if the source file exists ---
    if not os.path.exists(SOURCE_FILENAME):
        print(f"❌ Error: Source file '{SOURCE_FILENAME}' not found.")
        print("Please make sure this script is in the same folder as your data file.")
        return

    # --- 2. Get and validate user input ---
    while True:
        try:
            prompt = f"Please enter the new sample interval in seconds (must be a multiple of {ORIGINAL_INTERVAL_SECONDS}): "
            new_interval_sec = int(input(prompt))
            
            if new_interval_sec <= 0:
                print("Interval must be a positive number. Please try again.")
                continue
                
            if new_interval_sec % ORIGINAL_INTERVAL_SECONDS != 0:
                print(f"Error: Interval must be a multiple of {ORIGINAL_INTERVAL_SECONDS}. Please try again.")
                continue
                
            break # Exit loop if input is valid
        except ValueError:
            print("Invalid input. Please enter a whole number.")

    # --- 3. Calculate the step and define output filename ---
    step = new_interval_sec // ORIGINAL_INTERVAL_SECONDS
    output_filename = f"resampled_data_{new_interval_sec}s.csv"
    
    print(f"\nResampling data to a {new_interval_sec}-second interval.")
    print(f"This means we will take every {step}th row from the source file.")
    print("Processing...")

    try:
        with open(SOURCE_FILENAME, 'r', newline='') as infile, \
             open(output_filename, 'w', newline='') as outfile:
            
            reader = csv.reader(infile)
            writer = csv.writer(outfile)
            
            header = next(reader)
            writer.writerow(header)
            
            rows_written = 0
            
            # use enumerate to easily select every step row
            for i, row in enumerate(reader):
                if i % step == 0:
                    writer.writerow(row)
                    rows_written += 1
                    
        print(f"Success! Generated '{output_filename}' with {rows_written} data points.")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")


if __name__ == "__main__":
    resample_csv()