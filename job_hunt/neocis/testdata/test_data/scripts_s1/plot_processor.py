# plot_generator.py
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import sys
import argparse
import numpy as np
from pathlib import Path

def create_single_plot(df, encoder, accuracy_limit, output_path):
    """Create plot for a single encoder."""
    plt.figure(figsize=(10, 6))
    
    mask_pass = df['error'] <= accuracy_limit
    mask_fail = df['error'] > accuracy_limit
    
    plt.scatter(df[mask_pass]['gt'], df[mask_pass]['error'], 
               color='blue', label='Passed', alpha=0.6)
    plt.scatter(df[mask_fail]['gt'], df[mask_fail]['error'], 
               color='red', label='Failed', alpha=0.6)
    
    plt.axhline(y=accuracy_limit, color='r', linestyle='--', 
               label='Accuracy Limit')
    
    plt.title(f'Accuracy vs Ground Truth - Encoder {encoder}')
    plt.xlabel('Ground Truth (degrees)')
    plt.ylabel('Accuracy Error (degrees)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()

def create_combined_plot(all_data, accuracy_limit, output_path):
    """Create combined plot for all encoders."""
    plt.figure(figsize=(12, 8))
    
    colors = plt.cm.tab10(np.linspace(0, 1, len(all_data)))
    
    for (encoder, df), color in zip(all_data.items(), colors):
        plt.scatter(df['gt'], df['error'], 
                   label=f'Encoder {encoder}', 
                   alpha=0.6,
                   color=color,
                   s=30)
    
    plt.axhline(y=accuracy_limit, color='r', linestyle='--', 
               label='Accuracy Limit', linewidth=2)
    
    plt.title('Accuracy vs Ground Truth - All Encoders')
    plt.xlabel('Ground Truth (degrees)')
    plt.ylabel('Accuracy Error (degrees)')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()

def process_plots(folder_path, plot_type):
    """
    Generate plots based on command line arguments.
    
    Args:
        folder_path (str): Path to folder containing CSV files
        plot_type (str): Either "-all" or encoder number
    """
    try:
        # Get all CSV files
        csv_files = glob.glob(os.path.join(folder_path, "accuracy_*.csv"))
        if not csv_files:
            print(f"No accuracy CSV files found in {folder_path}")
            return False

        accuracy_limit = 0.01
        all_data = {}

        # Process all files
        for file in csv_files:
            encoder = file.split('_e')[-1].split('.')[0]
            df = pd.read_csv(file)
            df['error'] = abs(df['gt'] - df['enc'])
            all_data[encoder] = df

        if plot_type.lower() == "all":
            # Create output directory
            output_dir = Path("accuracy_out")
            output_dir.mkdir(exist_ok=True)
            
            # Generate individual plots
            for encoder, df in all_data.items():
                output_path = output_dir / f'accuracy_test_encoder_{encoder}.jpg'
                create_single_plot(df, encoder, accuracy_limit, output_path)
            
            # Generate combined plot
            output_path = output_dir / 'accuracy_test_combined.jpg'
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
        description='Generate accuracy plots for encoder data'
    )
    parser.add_argument('folder_path', 
                       help='Path to folder containing accuracy test CSV files')
    parser.add_argument('plot_type',
                       help='Either "all" for all encoders or encoder number',
                       type=str)  # Explicitly specify string type
    
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