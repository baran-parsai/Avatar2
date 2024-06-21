import argparse
from FaceRecognizer import FaceRecognizer

def build_database(data_loc, json_loc, output_loc):
    print("Building database")
    FaceRecognizer.encode_known_faces(data_loc, json_loc, output_loc)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build a face recognition database.")
    parser.add_argument('--source', type=str, default='/home/baranparsai/training', help='Directory where the source images can be found.')
    parser.add_argument('--destination', type=str, default='/home/baranparsai/Documents/Avatar2/scenarios/hearing_clinic', help='Directory where to save the pkl and json files.')

    args = parser.parse_args()

    data_location = args.source
    json_location = args.destination + '/faces/faces.json'  
    output_location = args.destination + '/faces/faces.pkl'  

    build_database(data_location, json_location, output_location)
