import os
import cv2
import faiss
import torch
import onnxruntime as rt
import torchvision.transforms as T
import pdb

from similarity.models.eff_net import EfficientNet


class Similarity(object):
    def __init__(self, cfg, device):
        print('Loading similarity model...')
        if os.path.exists(cfg.weights):
            # self.model = EfficientNet.from_pretrained(cfg.architecture, weights_path=cfg.weights)
            self.sess = rt.InferenceSession(cfg.weights)
        else:
            raise Exception('Model not found at ' + cfg.weights)
        # self.model = self.model.to(device)
        # self.model.eval()

        #pdb.set_trace()
        self.ref_features = faiss.read_index(cfg.ref_feats)  # torch.load(cfg.ref_feats, map_location=device)
        self.ref_labels = torch.load(cfg.ref_feats.replace('features', 'labels').replace('index', 'pt'), map_location=device)
        self.ref_skus = [x.strip() for x in open(cfg.ref_feats.replace('features', 'upcs').replace('index', 'txt')).readlines()]

        self.image_size = cfg.image_size
        # self.transform = T.Compose([
        #     T.ToPILImage(),
        #     T.Resize((self.image_size, self.image_size)),
        #     T.ToTensor(),
        #     # T.Normalize(mean=[0.48501, 0.45795, 0.4076], std=[0.2289, 0.2240, 0.2250]),
        # ])

        self.device = device

    def prepare_crop( img, obj):
        if 'confidence' not in obj:
            obj['confidence'] = 1
        xmin = int(obj['bndbox']['xmin'])
        xmax = int(obj['bndbox']['xmax'])
        ymin = int(obj['bndbox']['ymin'])
        ymax = int(obj['bndbox']['ymax'])
        # crop = 255.0*self.transform(img[ymin:ymax, xmin:xmax, :]).to(self.device)
        crop = cv2.resize(img[ymin:ymax, xmin:xmax, :], (image_size, image_size)).transpose(2, 0, 1)
        return crop

    def display_results(img, dets, colours=None):
        #pdb.set_trace()
        dist = 0
        dist_old = 0
        yold = 0
        item_count = 0
        diff_sum = 0
        yave_array = []
        if colours is None:
            colours = np.random.randint(255, (3, len(dets)))
        for obj in dets:
            xmin = int(float(obj['bndbox']['xmin']))
            xmax = int(float(obj['bndbox']['xmax']))
            ymin = int(float(obj['bndbox']['ymin']))
            ymax = int(float(obj['bndbox']['ymax']))
            if obj['name'] == 'shelf':
                img = cv2.rectangle(img, (xmin, ymin), (xmax, ymax),
                                    color= (0,0,255),
                                    thickness=16) 
                yave = (ymax + ymin)/2                                      
                yave_array.insert(item_count, yave)
                #img = cv2.putText(img, obj['sku'], (xmin, ymax), 1, 3,
                #        color=colours[self.ref_labels[obj['class']].item() % len(colours)],
                #thickness=4)
              
            elif obj['name'] == 'label':
                img = cv2.rectangle(img, (xmin, ymin), (xmax, ymax), color=colours[-1], thickness=4)

       
        #uncomment dff_sum when running shelf detection  
        '''                                  
        yave_array.sort()
        yave_array_len = len(yave_array)
        for i in range(1,len(yave_array)):
            spacing = yave_array[i] - yave_array[i-1]
            if (spacing > 200):
                diff_sum = diff_sum + spacing
            else:
                yave_array_len = yave_array_len - 1    
  
        diff_sum = diff_sum/(yave_array_len)       
        print("Shelf spacing = ",diff_sum)                  
        '''                    

        return img
