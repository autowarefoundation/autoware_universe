# VadNode Design

- code: [vad_node.cpp](../src/vad_node.cpp), [vad_node.hpp](../include/autoware_tensorrt_vad/vad_node.hpp)

## 責務

- ROS topicをsubscribeしてcallback関数を実行し、`VadInputTopicData`に詰め込む

## 処理のflowchart

```mermaid
flowchart TD
    Start([Anchor Topic Subscribed]) --> AnchorCallback[anchor_callback]
    
    subgraph AnchorCallbackScope["anchor_callback()"]
        AnchorCallback --> CheckReady{Is VadInputTopicData synchronization ready?}
        
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
                ConvertInput --> VadModelInfer[VadModel::infer]
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
    
    %% Links to source code
    click AnchorCallback "../src/vad_node.cpp" "anchor_callback() implementation"
    click TriggerInferenceEntry "../src/vad_node.cpp" "trigger_inference() implementation"
    click ExecuteInferenceEntry "../src/vad_node.cpp" "execute_inference() implementation"
    click Publish "../src/vad_node.cpp" "publish() implementation"
    
    style AnchorCallbackScope fill:#f0f8ff,stroke:#4682b4,stroke-width:2px,color:#000000
    style TriggerInferenceScope fill:#fff8dc,stroke:#daa520,stroke-width:2px,color:#000000
    style ExecuteInferenceScope fill:#f5f5dc,stroke:#8b4513,stroke-width:2px,color:#000000
```

### 関数の役割と実装

- [`anchor_callback()`](../src/vad_node.cpp): anchor topic(最後にsubscribeされる画像topic)を受け取った際に起動するcallback
- [`trigger_inference()`](../src/vad_node.cpp): データの同期チェックをし、inference処理をtrigger
- [`execute_inference()`](../src/vad_node.cpp): VADのinference処理を実行。`VadInputTopicData`から`VadOutputTopicData`を推論する。
- [`publish()`](../src/vad_node.cpp): 推論結果をROS topicとしてpublish

## TODO

- subscribeする際にcallbackを使いすぎず、takeを用いる
- ROS parameterからのconfigの作成、publisherとsubscriberの作成、callback関数、inferenceのtriggerとexecute、publishといろいろなことを行っている。読みづらくなってきた場合は、それぞれの責務ごとにclassを分割する必要がある。
