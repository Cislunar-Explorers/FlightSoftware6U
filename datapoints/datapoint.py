
class DataPoint():
    
    def __init__(self, codec_type: str, sqlach_type: str, data_func):
        self.codec_type = codec_type
        self.sqlach_type = sqlach_type 
        self.data_func = data_func

    def getData(self):
        self.data_func()

