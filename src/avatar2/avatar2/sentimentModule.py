import io
from .features import *
import ament_index_python.packages
from .functions import predict_on_model

class Sentiment():
    def __init__(self, path='/home/walleed/Avatar_2/src/avatar2/classification_model/model'):
        # self.declare_parameter('model_path', '/classification_model/model')
        package_path = ament_index_python.packages.get_package_share_directory('avatar2')
        # self._model_path = os.path.join(package_path, 'classification_model', 'model')
        # self._model_path = package_path + '/classification_model/model'
        self._model_path = path
        print("Sentiment tool created")

    def analyze(self, data):
        wav_file = io.BytesIO(bytes.fromhex(data))
        with wav_file as source:
            score = predict_on_model(source, self._model_path)[0]
        return score
