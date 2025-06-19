# Import the InferencePipeline object
import cv2
from inference import InferencePipeline


def my_sink(result, video_frame):
    if result.get("output_image"):  # Display an image from the workflow response
        output = result.get("output_image").numpy_image #Save image to output and do with it as you wish
        height = output.shape[0]
        width = output.shape[1]
        text = cv2.putText(output, (str(width) + "x" + str(height)), (580, 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        cv2.imshow("Workflow Image", output) #Shows chosen image
        if cv2.waitKey(1) == ord("q"): #Selfexplanatory
            pipeline.terminate()
    print(result)  # do something with the predictions of each frame


# initialize a pipeline object
pipeline = InferencePipeline.init_with_workflow(
    api_key="VD9BLusLGWoKvrez3ufK",
    workspace_name="dtu-ywbko",
    workflow_id="detect-count-and-visualize",
    video_reference=0,  # Path to video, device id (int, usually 0 for built in webcams), or RTSP stream url
    max_fps=240,
    on_prediction=my_sink
)
pipeline.start()  # start the pipeline
pipeline.join()  # wait for the pipeline thread to finish
