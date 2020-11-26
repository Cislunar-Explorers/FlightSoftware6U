from elasticsearch import Elasticsearch, client
from datetime import datetime
#import serialization

#Cluster client
es = Elasticsearch(
    cloud_id = "alex-cislunar-test:dXMtZWFzdDQuZ2NwLmVsYXN0aWMtY2xvdWQuY29tJGRiM\
    2ZkNTAzODk3MDRmM2E4MjYzZGViMzc0YzY0NGIyJGUxMzI1MGUxZGM5MzQ3MjhhMDQwYjc1NWM3Y\
    TZlZGQz",
    http_auth=("elastic", "OSN8cIqk8SnG2GCCJu7b1Yff"),
)

#Manages indices in cluster es
esIndexClient = client.IndicesClient(es)

#Creates a new ElasticSearch index
def createIndex(name):
    if not esIndexClient.exists(name):
        esIndexClient.create(name)
        
#Creates a timestamped document in the specified index
def createDocument(indexName: str, **kwargs):
    
    try:
        docBody = {}
        docBody['timestamp'] = str(datetime.now())

        for field in kwargs:
            docBody[field] = kwargs[field]
        
        es.index(index = indexName, id= 7, body = docBody)

    except:
        print('no')
    

#createDocument('gyro', gyro1 = 50, gyro2 = 123)
