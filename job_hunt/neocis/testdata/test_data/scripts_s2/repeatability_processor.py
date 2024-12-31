# repeatability_processor.py

import pandas as pd
import glob
import os
import sys
import argparse
from pathlib import Path

def process_repeatability_data(folder_path):
    """
    Process encoder repeatability data and generate statistics CSV.
    
    Args:
        folder_path (str): Path to folder containing repeatability test CSV files
    """
    try:
        results = []
        
        # Get all CSV files in the folder
        csv_files = glob.glob(os.path.join(folder_path, "repeatability_*.csv"))
        if not csv_files:
            print(f"No repeatability CSV files found in {folder_path}")
            return False
            
        # Process each encoder file
        for file in csv_files:
            encoder = file.split('_e')[-1].split('.')[0]
            print(f"Processing encoder {encoder}...")
            
            # Read and process data
            df = pd.read_csv(file)
            
            # Calculate errors for both positions
            df['p1_error'] = abs(df['p1_gt'] - df['p1_enc'])
            df['p2_error'] = abs(df['p2_gt'] - df['p2_enc'])
            
            # Calculate statistics
            accuracy_limit = 0.01  # ±0.01° specification
            passed_count = sum((df['p1_error'] <= accuracy_limit) & 
                             (df['p2_error'] <= accuracy_limit))
            failed_count = len(df) - passed_count
            
            result = {
                'encoder': encoder,
                'passed': passed_count,
                'failed': failed_count,
                'p1_repeatability': df['p1_error'].std(),
                'p2_repeatability': df['p2_error'].std(),
                'acceptability': 'Yes' if failed_count == 0 else 'No'
            }
            
            results.append(result)
        
        # Save results to CSV
        results_df = pd.DataFrame(results)
        output_dir = Path("repeatability_out")
        output_dir.mkdir(exist_ok=True)
        results_df.to_csv(output_dir / 'accuracy_kpi_results.csv', index=False)
        print(f"\nResults saved to accuracy_kpi_results.csv")
        return True
        
    except Exception as e:
        print(f"Error processing data: {str(e)}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Process encoder repeatability test data'
    )
    parser.add_argument('folder_path', 
                       help='Path to folder containing repeatability test CSV files')
    
    args = parser.parse_args()
    
    folder_path = Path(args.folder_path)
    if not folder_path.exists() or not folder_path.is_dir():
        print(f"Error: Invalid folder path '{folder_path}'")
        sys.exit(1)
    
    success = process_repeatability_data(str(folder_path))
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()