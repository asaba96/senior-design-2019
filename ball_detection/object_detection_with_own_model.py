# External imports
import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
import cv2
from distutils.version import StrictVersion
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

# Internal Imports
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# Globals 
MODEL_NAME= 'inference_graph'
PATH_TO_FROZEN_GRAPH= 'neural_networks/ball_rcnn_faster/output' + '/frozen_inference_graph.pb'
PATH_TO_LABELS= 'neural_networks/ball_rcnn_faster/training/object-detection.pbtxt'
PATH_TO_VIDEO= 'neural_networks/ball_rcnn_faster/testing4.mp4'
ALLOW_RECORD= True
PATH_TO_RECORD= 'neural_networks/ball_rcnn_faster/testing42.avi'
FRAME_SKIP= 3

# Build detection graph
detection_graph= tf.Graph()
with detection_graph.as_default():
  od_graph_def= tf.GraphDef()
  with tf.gfile.GFile( PATH_TO_FROZEN_GRAPH, 'rb') as fid:
    serialized_graph= fid.read()
    od_graph_def.ParseFromString( serialized_graph)
    tf.import_graph_def( od_graph_def, name= '')

# Build label map for predicting classes
category_index = label_map_util.create_category_index_from_labelmap( PATH_TO_LABELS, use_display_name= True)

# Get source video
cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture( PATH_TO_VIDEO)
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))

# Setup output recording
if ALLOW_RECORD: recorder = cv2.VideoWriter( PATH_TO_RECORD, 
    cv2.VideoWriter_fourcc('M','J','P','G'), 10, (frame_width,frame_height))

# Setup ball dimensions
ball_width= .07
ball_height= .07

corner_points3D= np.array([[
    [-ball_width / 2, ball_height / 2, 0],
    [ball_width / 2, ball_height / 2, 0],
    [ball_width / 2, -ball_height / 2, 0],
    [-ball_width / 2, -ball_height / 2, 0]
]])

def get_coordinates( output_dict, frame_width, frame_height):
    box, cla, sco= output_dict['detection_boxes'][0], output_dict['detection_classes'][0], output_dict['detection_scores'][0]
    ymin_norm, xmin_norm, ymax_norm, xmax_norm= box
    xmin, xmax= xmin_norm * frame_width, xmax_norm * frame_width
    ymin, ymax= ymin_norm * frame_height, ymax_norm * frame_height
    return np.array([
        [xmin, ymax],
        [xmax, ymax],
        [xmax, ymin],
        [xmin, ymin]
    ])

def get_distance( sco, corner_points3D, corners_array, frame_height, frame_width):
    if sco[0] < .50:
        return np.nan, np.nan, np.nan
    CAMERA_EX= 135 #30
    const_circ_dia= 0.07
    const_circ_dia_at_1m= CAMERA_EX
    check=(corners_array[1][0] - corners_array[0][0]) / (corners_array[0][1] - corners_array[2][1])
    if not 0.5 < check < 1.5:
        return np.nan, np.nan, np.nan
    this_circ_dia_pix= (corners_array[1][0] - corners_array[0][0] + corners_array[0][1] - corners_array[2][1]) / 2
    z_trans=const_circ_dia_at_1m/this_circ_dia_pix
    rad= this_circ_dia_pix/const_circ_dia_at_1m * CAMERA_EX/2
    center= ((corners_array[1][0] + corners_array[0][0])/2, (corners_array[0][1] + corners_array[2][1])/2)
    camera_center= (frame_width/2, frame_height/2)
    x_trans_pix= center[0]-camera_center[0]
    y_trans_pix= center[1]-camera_center[1]
    x_trans= x_trans_pix / CAMERA_EX * const_circ_dia * z_trans
    y_trans= y_trans_pix / CAMERA_EX * const_circ_dia * z_trans
    return x_trans, y_trans, z_trans

def run_inference_for_single_image( image, graph):
    if 'detection_masks' in tensor_dict:
        # The following processing is only for single image
        detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
        detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])

        # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
        real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
        detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
        detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
        detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
            detection_masks, detection_boxes, image.shape[0], image.shape[1])
        detection_masks_reframed = tf.cast(
            tf.greater(detection_masks_reframed, 0.5), tf.uint8)
        
        # Follow the convention by adding back the batch dimension
        tensor_dict['detection_masks'] = tf.expand_dims(
            detection_masks_reframed, 0)
    image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')

    # Run inference
    output_dict = sess.run(tensor_dict,
                            feed_dict={image_tensor: np.expand_dims(image, 0)})

    # all outputs are float32 numpy arrays, so convert types as appropriate
    output_dict['num_detections'] = int(output_dict['num_detections'][0])
    output_dict['detection_classes'] = output_dict[
        'detection_classes'][0].astype(np.uint8)
    output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
    output_dict['detection_scores'] = output_dict['detection_scores'][0]
    if 'detection_masks' in output_dict:
        output_dict['detection_masks'] = output_dict['detection_masks'][0]
    return output_dict

try:
    with detection_graph.as_default():
        with tf.Session() as sess:
            # Get handles to input and output tensors
            ops = tf.get_default_graph().get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            tensor_dict = {}
            for key in [
                'num_detections', 'detection_boxes', 'detection_scores',
                'detection_classes', 'detection_masks'
            ]:
                tensor_name = key + ':0'
                if tensor_name in all_tensor_names:
                    tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                    tensor_name)

            frame_count=0
            avg= list()
            while True:
                frame_count+=1
                ret, image_np = cap.read()
                if not frame_count % FRAME_SKIP == 0:
                    continue
                image_np= cv2.resize( image_np, (int( frame_width), int( frame_height)))
                
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                
                # Actual detection.
                output_dict = run_inference_for_single_image(image_np, detection_graph)
                
                # Visualization of the results of a detection.
                vis_util.visualize_boxes_and_labels_on_image_array(
                    image_np,
                    output_dict['detection_boxes'],
                    output_dict['detection_classes'],
                    output_dict['detection_scores'],
                    category_index,
                    instance_masks=output_dict.get('detection_masks'),
                    use_normalized_coordinates=True,
                    line_thickness=2)
                
                # Localization
                corners_array= get_coordinates( output_dict, frame_width, frame_height)
                x_trans, y_trans, z_trans= get_distance( output_dict['detection_scores'], corner_points3D, corners_array, frame_height, frame_width)
                avg.append( output_dict['detection_scores'][0])
                print( sum( avg) / len( avg))
                
                """
                cm= np.eye( 3)
                dc= np.zeros(( 5,1))
                retval, rvec, tvec= cv2.solvePnP( corner_points3D.astype( float), 
                    corners_array.astype( float), cm, dc)
                print( tvec[0], tvec[1])
                """
                s= f"Localized (cm): X:{x_trans * 100:.2f}, Y:{y_trans * 100:.2f}, Z:{z_trans * 100:.2f}"
                cv2.putText(image_np,s,(30,80), cv2.FONT_HERSHEY_SIMPLEX, .85, (255, 255, 255), 2, cv2.LINE_AA)

                cv2.imshow('object_detection', cv2.resize( image_np, ( int(frame_width/2), int( frame_height/2))))
                
                if ALLOW_RECORD: recorder.write(image_np)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    cap.release()
                    if ALLOW_RECORD: recorder.release()
                    cv2.destroyAllWindows()
                    break
except Exception as e:
    print(e)
    cap.release()
    if ALLOW_RECORD: recorder.release()
