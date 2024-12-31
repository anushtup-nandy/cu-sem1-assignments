# repeatability_plot_processor.py

import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys
import argparse
import numpy as np
from pathlib import Path

def create_single_plot(df, encoder, accuracy_limit, output_path):
    """Create bar plot for a single encoder."""
    plt.figure(figsize=(10, 6))
    
    # Calculate errors
    p1_error = df['p1_error'].std()
    p2_error = df['p2_error'].std()
    
    # Create bar plot
    positions = ['Position 1 (0°)', 'Position 2 (90°)']
    values = [p1_error, p2_error]
    colors = ['blue' if v <= accuracy_limit else 'red' for v in values]
    
    plt.bar(positions, values, color=colors, alpha=0.6)
    
    # Add limit line
    plt.axhline(y=accuracy_limit, color='r', linestyle='--',
               label='Repeatability Limit')
    
    plt.title(f'Position Repeatability - Encoder {encoder}')
    plt.ylabel('Repeatability (Standard Deviation)')
    plt.grid(True, alpha=0.3)
    plt.legend()
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()

def create_combined_plot(all_data, accuracy_limit, output_path):
    """Create combined bar plot for all encoders."""
    plt.figure(figsize=(15, 8))
    
    n_encoders = len(all_data)
    x = np.arange(n_encoders)
    width = 0.35
    
    p1_values = []
    p2_values = []
    encoder_labels = []
    
    for encoder, df in all_data.items():
        p1_values.append(df['p1_error'].std())
        p2_values.append(df['p2_error'].std())
        encoder_labels.append(f'Encoder {encoder}')
    
    # Create grouped bar plot
    plt.bar(x - width/2, p1_values, width, label='Position 1 (0°)',
            color=['blue' if v <= accuracy_limit else 'red' for v in p1_values])
    plt.bar(x + width/2, p2_values, width, label='Position 2 (90°)',
            color=['blue' if v <= accuracy_limit else 'red' for v in p2_values])
    
    plt.axhline(y=accuracy_limit, color='r', linestyle='--',
               label='Repeatability Limit')
    
    plt.xlabel('Encoders')
    plt.ylabel('Repeatability (Standard Deviation)')
    plt.title('Position Repeatability - All Encoders')
    plt.xticks(x, encoder_labels)
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()

def process_plots(folder_path, plot_type):
    """
    Generate plots based on command line arguments.
    
    Args:
        folder_path (str): Path to folder containing CSV files
        plot_type (str): Either "all" or encoder number
    """
    try:
        # Get all CSV files
        csv_files = glob.glob(os.path.join(folder_path, "repeatability_*.csv"))
        if not csv_files:
            print(f"No repeatability CSV files found in {folder_path}")
            return False

        accuracy_limit = 0.01
        all_data = {}

        # Process all files
        for file in csv_files:
            encoder = file.split('_e')[-1].split('.')[0]
            df = pd.read_csv(file)
            df['p1_error'] = abs(df['p1_gt'] - df['p1_enc'])
            df['p2_error'] = abs(df['p2_gt'] - df['p2_enc'])
            all_data[encoder] = df

        if plot_type.lower() == "all":
            # Create output directory
            output_dir = Path("repeatability_out")
            output_dir.mkdir(exist_ok=True)
            
            # Generate individual plots
            for encoder, df in all_data.items():
                output_path = output_dir / f'repeatability_test_encoder_{encoder}.jpg'
                create_single_plot(df, encoder, accuracy_limit, output_path)
            
            # Generate combined plot
            output_path = output_dir / 'repeatability_test_combined.jpg'
            create_combined_plot(all_data, accuracy_limit, output_path)
            print(f"All plots saved in {output_dir}/")
            
        else:
            # Generate plot for specific encoder
            if plot_type not in all_data:
                print(f"Error: Encoder {plot_type} not found")
                return False
                
            create_single_plot(all_data[plot_type], plot_type,
                             accuracy_limit, "plot_test.jpg")
            print("Plot saved as plot_test.jpg")
            
        return True
        
    except Exception as e:
        print(f"Error generating plots: {str(e)}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Generate repeatability plots for encoder data'
    )
    parser.add_argument('folder_path', 
                       help='Path to folder containing repeatability test CSV files')
    parser.add_argument('plot_type',
                       help='Either "all" for all encoders or encoder number',
                       type=str)
    
    args = parser.parse_args()
    
    folder_path = Path(args.folder_path)
    if not folder_path.exists() or not folder_path.is_dir():
        print(f"Error: Invalid folder path '{folder_path}'")
        sys.exit(1)
    
    plot_type = "all" if args.plot_type in ["-all", "all"] else args.plot_type
    success = process_plots(str(folder_path), plot_type)
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()