# VadNode Design

- code: [vad_node.cpp](../src/vad_node.cpp), [vad_node.hpp](../include/autoware_tensorrt_vad/vad_node.hpp)

## 責務

- ROS topicをsubscribeしてcallback関数を実行し、`VadInputTopicData`に詰め込む

## Processing Flowchart

```mermaid
flowchart TD
    Start([Anchor Topic Subscribed]) --> CheckReady
    
    subgraph AnchorCallbackScope["anchor_callback()"]
        CheckReady{Is VadInputTopicData synchronization ready?}
        
        CheckReady -->|Yes| CheckDropped
        CheckReady -->|No| ErrorLog[Log error]
        
        subgraph TriggerInferenceScope["trigger_inference()"]
            CheckDropped{Are any topics<br/>dropped?}
            
            CheckDropped -->|Yes| FillDropped[fill_dropped_data]
            CheckDropped -->|No| CheckComplete{Are all required topics<br/>completed?}
            
            FillDropped --> CheckComplete
            
            CheckComplete -->|Yes| ConvertInput
            CheckComplete -->|No| ReturnNull[return nullopt]
            
            subgraph ExecuteInferenceScope["execute_inference()"]
                ConvertInput[Convert from VadInputTopicData to VadInputData]
                ConvertInput --> VadModelInfer[VadModel::infer]
                VadModelInfer --> ConvertOutput[Convert from VadOutputData to VadOutputTopicData]
                ConvertOutput --> GetOutput[return VadOutputTopicData]
            end
        end
        
        GetOutput --> Publish[Publish result topics]
        Publish --> Reset[Reset current frame data]
        ErrorLog --> Reset
        ReturnNull --> Reset
    end
    
    Reset --> End([End])
    
    style AnchorCallbackScope fill:#f0f8ff,stroke:#4682b4,stroke-width:2px,color:#000000
    style TriggerInferenceScope fill:#fff8dc,stroke:#daa520,stroke-width:2px,color:#000000
    style ExecuteInferenceScope fill:#f5f5dc,stroke:#8b4513,stroke-width:2px,color:#000000
```

### Function Roles

- [`anchor_callback()`](../src/vad_node.cpp): Callback triggered when receiving the anchor topic (the last subscribed image topic)
- [`trigger_inference()`](../src/vad_node.cpp): Checks data synchronization and triggers inference
- [`execute_inference()`](../src/vad_node.cpp): Executes VAD inference. Infers `VadOutputTopicData` from `VadInputTopicData`
- [`publish()`](../src/vad_node.cpp): Publishes inference results as ROS topics

## TODO

- subscribeする際にcallbackを使いすぎず、takeを用いる
- ROS parameterからのconfigの作成、publisherとsubscriberの作成、callback関数、inferenceのtriggerとexecute、publishといろいろなことを行っている。読みづらくなってきた場合は、それぞれの責務ごとにclassを分割する必要がある。
