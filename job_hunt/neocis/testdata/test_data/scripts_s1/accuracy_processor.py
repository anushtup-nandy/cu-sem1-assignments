# accuracy_calculator.py
import pandas as pd
import glob
import os
import sys
import argparse
from pathlib import Path

def process_accuracy_data(folder_path):
    """
    Process encoder accuracy data and generate statistics CSV.    
    Args:
        folder_path (str): Path to folder containing accuracy test CSV files
    """
    try:
        results = []
        
        # Get all CSV files in the folder
        csv_files = glob.glob(os.path.join(folder_path, "accuracy_*.csv"))
        if not csv_files:
            print(f"No accuracy CSV files found in {folder_path}")
            return False
            
        # Process each encoder file
        for file in csv_files:
            encoder = file.split('_e')[-1].split('.')[0]
            print(f"Processing encoder {encoder}...")
            
            df = pd.read_csv(file) #read and convert the csv into a dataframe
            df['error'] = abs(df['gt'] - df['enc'])
            
            accuracy_limit = 0.01
            passed_count = sum(df['error'] <= accuracy_limit)
            failed_count = sum(df['error'] > accuracy_limit)
            
            result = {
                'encoder': encoder,
                'passed': passed_count,
                'failed': failed_count,
                'max_error': df['error'].max(),
                'mean_error': df['error'].mean(),
                'min_error': df['error'].min(),
                'std_error': df['error'].std(),
                'q95_error': df['error'].quantile(0.95),
                'q5_error': df['error'].quantile(0.05),
                'acceptability': 'Yes' if failed_count == 0 else 'No'
            }
            results.append(result)
        
        # Save results to CSV
        results_df = pd.DataFrame(results)
        results_df.to_csv('./accuracy_out/accuracy_kpi_results.csv', index=False)
        print(f"\nResults saved to accuracy_kpi_results.csv")
        return True
        
    except Exception as e:
        print(f"Error processing data: {str(e)}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description='Process encoder accuracy test data and generate statistics'
    )
    parser.add_argument('folder_path', 
                       help='Path to folder containing accuracy test CSV files')
    
    args = parser.parse_args()
    
    folder_path = Path(args.folder_path)
    if not folder_path.exists() or not folder_path.is_dir():
        print(f"Error: Invalid folder path '{folder_path}'")
        sys.exit(1)
    
    success = process_accuracy_data(str(folder_path))
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()