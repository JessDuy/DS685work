import psycopg2, torch, torchvision, numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from torchvision import transforms as T

class ObjDet(Node):
    def __init__(self):
        super().__init__('objdet')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

        self.device = torch.device('cpu')
        self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights='DEFAULT').to(self.device).eval()
        self.embedder = torchvision.models.resnet50(weights='DEFAULT')
        self.embedder.fc = torch.nn.Identity()
        self.embedder = self.embedder.to(self.device).eval()

        self.det_tf = T.ToTensor()
        self.emb_tf = T.Compose([
            T.ToPILImage(),
            T.Resize((224,224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225])
        ])

        # DB (docker-compose service "db")
        self.conn = psycopg2.connect(host='db', dbname='rosmap', user='postgres', password='postgres')
        self.conn.autocommit = True

        # For grading tag: 'sim' default; use 'webcam' if you publish from cam_pub
        self.source = self.declare_parameter('source', 'sim').get_parameter_value().string_value

    def cb(self, msg: Image):
        try:
            import cv2
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        with torch.no_grad():
            det_out = self.model([self.det_tf(frame).to(self.device)])[0]

        boxes = det_out['boxes'].cpu().numpy().astype(int)
        scores = det_out['scores'].cpu().numpy()
        labels = det_out['labels'].cpu().numpy()

        mask = scores >= 0.6
        boxes, scores, labels = boxes[mask], scores[mask], labels[mask]

        for (x1,y1,x2,y2), s, c in zip(boxes, scores, labels):
            x1, y1 = max(0,x1), max(0,y1)
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue

            with torch.no_grad():
                emb = self.embedder(self.emb_tf(crop[:,:,::-1]).unsqueeze(0).to(self.device))
                emb = emb.squeeze(0).cpu().numpy().astype(np.float32)  # 2048-d

            self.store_detection(
                frame_id=msg.header.frame_id or 'camera_link',
                cls=str(int(c)), score=float(s),
                box=(int(x1),int(y1),int(x2),int(y2)),
                emb=emb, source=self.source
            )

    def store_detection(self, frame_id, cls, score, box, emb, source):
        with self.conn.cursor() as cur:
            cur.execute(
                """
                INSERT INTO detections
                (frame_id, class, score, x_min, y_min, x_max, y_max, embedding, source)
                VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s)
                """,
                (frame_id, cls, score, box[0], box[1], box[2], box[3], list(emb), source)
            )

def main():
    rclpy.init()
    node = ObjDet()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
