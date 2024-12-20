## Visual Tracker

### Workflow
```
if (init):
    featureDetection();
    undistortion();
else:
    trackPrediction();
    if not success :
        trackPreviousFeatures();
    saveTrackingResult();
    featureDetection();
```

### TODO
- [ ] degeneration detection
- [X] save and get configuration file
- [X] move unnecessary function and parameters to private domain
- [X] re-organize const expression
- [X] test
- [ ] is flow-back checking necessary?
- [X] finish comments
- [X] documentation

### Notices