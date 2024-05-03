"""Simulate Protocols to get csv of module actions."""
from opentrons.cli import analyze
import argparse
import os

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description= "Simulate protocols in local directory.")
    parser.add_argument(
        "storage_directory",
        metavar="STORAGE_DIRECTORY",
        type=str,
        nargs=1,
        help="Path to protocols.",
    )
    args = parser.parse_args()
    storage_directory = args.storage_directory[0]
    list_of_protocols = os.listdir(storage_directory)
    for protocol in list_of_protocols:
        if protocol.endswith(".py"):
            protocol_path = os.path.join(storage_directory, protocol)
            analyze(protocol_path)

        
    