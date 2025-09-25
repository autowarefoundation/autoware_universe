# VadNode Design

- code: [vad_node.cpp](../src/vad_node.cpp), [vad_node.hpp](../include/autoware_tensorrt_vad/vad_node.hpp)

## 責務

- ROS topicをsubscribeしてcallback関数を実行し、`VadInputTopicData`に詰め込む

## 処理のflowchart

```mermaid
flowchart TD
    Start([Anchor Topic Subscribed]) --> AnchorCallback[anchor_callback]
    
    subgraph AnchorCallbackScope["anchor_callback()"]
        AnchorCallback --> CheckReady{Is vad_input_topic_data_current_frame_<br/>synchronization ready?}
        
        CheckReady -->|Yes| TriggerInferenceEntry[trigger_inference]
        CheckReady -->|No| ErrorLog[Log error]
        
        subgraph TriggerInferenceScope["trigger_inference()"]
            TriggerInferenceEntry --> CheckDropped{Are any topics<br/>dropped?}
            
            CheckDropped -->|Yes| FillDropped[fill_dropped_data]
            CheckDropped -->|No| CheckComplete{Are all required topics<br/>completed?}
            
            FillDropped --> CheckComplete
            
            CheckComplete -->|Yes| ExecuteInferenceEntry[execute_inference]
            CheckComplete -->|No| ReturnNull[return nullopt]
            
            subgraph ExecuteInferenceScope["execute_inference()"]
                ExecuteInferenceEntry --> ConvertInput[Convert to VadInputData]
                ConvertInput --> VadModelInfer[VadModel.infer]
                VadModelInfer --> ConvertOutput[Convert to VadOutputTopicData]
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

- `execute_inference`と`trigger_inference`の違い
  - `trigger_inference`: データの同期チェックと前処理を担当
  - `execute_inference`: 実際のVAD推論処理を実行

## TODO

- subscribeする際にcallbackを使いすぎず、takeを用いる
- ROS parameterからのconfigの作成、publisherとsubscriberの作成、callback関数、inferenceのtriggerとexecute、publishといろいろなことを行っている。読みづらくなってきた場合は、それぞれの責務ごとにclassを分割する必要がある。
