## Mobile Robot Software

Dynamic Worksite Models with Mobile Robots



## Training on Google Cloud
Connect to the Google Cloud
```sh
gcloud compute ssh --zone "us-west1-b" "maxkferg@mobile-robot-training"
```

Copy the files back to the current directory
```sh
# Entire results directory
gcloud compute scp --recurse --zone "us-west1-b" "maxkferg@mobile-robot-training:~/apps/mobile-robot-software/vision/assets/output/" .

# Individual files
gcloud compute scp --recurse --zone "us-west1-b" "maxkferg@mobile-robot-training:~/apps/mobile-robot-software/vision/assets/output/rgb/*.png" .
```