# Design

VAD ROS Node設計時に重要視したconceptについて説明します。

- ROSの領域とCUDAの領域の分離
  - ROS topicの型が変更されても、CUDAを使った実装には影響しない
  - CUDAのversionやinterfaceが変更されても、ROS Nodeには影響しない
- onnxに関係なく変えても良いROS parameterと、onnxに紐付いていてonnxとセットで変更が必要なROS parameterの分離
  - onnxに関係しているparameterは[`ml_package_vad_tiny.param.yaml`](../config/ml_package_vad_tiny.param.yaml)に追加
  - onnxに関係なく変えても良いROS parameterは[`vad_tiny.param.yaml`](../config/vad_tiny.param.yaml)に追加
    - object class remappingについては[`object_class_remapper.param.yaml`](../config/object_class_remapper.param.yaml)に追加
      - [`autoware_bevfusion`](../../../perception/autoware_bevfusion/README.md)の前例を踏襲。
- 「Autowareで使われる`camera_id`変更」という拡張に対して開いた設計
  - front cameraのidが`0`から`1`になったとしても、大きな設計変更を入れずに対応できるようにする

## ROSの世界とCUDAの世界の分離

VADの処理は「ROS/Autowareの世界」と「CUDAによる推論処理の世界」に明確に分離されています。

- **ROS側の責務**:

  - ROS topicのsubscribe/publish
    - topicのdrop, syncの確認
  - Autowareとの結合(publish/subscribe)

- **Interfaceの責務**:

  - input側
    - 座標変換
    - ROS Topicから`VadInputData`への変換処理
    - camera id mapping
  - output側
    - 座標変換
    - `VadOutputData`からROS Topicへの変換処理

- **VAD（CUDA）側の責務**:
  - camera画像の前処理(CUDA preprocessing)
  - VADの推論
    - `VadInputData`から`VadOutputData`を推論
  - 出力の後処理(CUDA postprocessing)

インターフェース（`VadInterface`）でAutowareとVADのTensorRT実装を橋渡ししており、互いの変更の影響範囲を最小限にするように設計されています。

---

### Dependancy Graph

```mermaid
graph TD
    VadNode["VadNode"]
    VadInterface["VadInterface"]
    VadInputTopicData(["VadInputTopicData"])
    VadOutputTopicData(["VadOutputTopicData"])
    VadModel["VadModel"]
    VadInputData(["VadInputData"])
    VadOutputData(["VadOutputData"])
    VadNode --> VadInputTopicData
    VadNode --> VadOutputTopicData
    VadInterface --> VadInputTopicData
    VadInterface --> VadOutputTopicData
    VadInterface --> VadInputData
    VadInterface --> VadOutputData
    VadModel --> VadInputData
    VadModel --> VadOutputData

    style VadNode fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadModel fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
    style VadInterface fill:#633974,stroke:#512E5F,stroke-width:2px,color:#FFFFFF;

    style VadInputTopicData fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadOutputTopicData fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;

    style VadInputData fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
    style VadOutputData fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
```

- `VadInterface`: ROS環境とVAD間のインターフェース

- `VadInputData`, `VadOutputData`: 推論処理で使用されるデータ構造

- `VadModel`: CUDA, TensorRTを用いた推論モデル

`VadInterface`はROS側の`VadInputTopicData`,`VadOutputTopicData`とVAD側のデータである`VadInputData`および`VadOutputData`に依存し、これらのデータ形式間の変換を担います。

`VadModel`はVADドメイン内部の`VadInputData`と`VadOutputData` **のみ** に依存します。

この依存構造により、`VadInterface`がROSの世界とVADの世界の間の緩衝材として機能し、双方の影響と責任範囲を分離して変更の影響を限定します。具体的に言えば，以下のような状態を実現することを期待します．

- ROS・Autoware側に変更が必要な場合(ROS topicの名称・内容変更など)，`VadModel`, `VadInputData`, `VadOutputData` にまったく変更を加えなくても良い
- CUDA, TensorRT側に変更が必要な場合(CUDA, TensorRTのバージョン変更など)，`VadInputTopicData`,`VadOutputTopicData` にまったく変更を加えなくても良い

---

### 処理フロー図

```mermaid
flowchart TD
    subgraph "ROS Domain (input)"
  VadNode_sub[VadNode::execute_inference]
  VadInputTopicData([VadInputTopicData])
        VadInterface[VadInterface]
    end

    subgraph "VAD Domain"
        VadInputData([VadInputData])
        VadModel::infer[VadModel::infer]
        VadOutputData([VadOutputData])
    end

    subgraph "ROS Domain (output)"
  VadInterface2[VadInterface]
        VadOutputTopicData([VadOutputTopicData])
        VadNode_pub[VadNode::publish]
    end

 VadNode_sub --> VadInputTopicData
 VadInputTopicData --> VadInterface

    VadInterface --> VadInputData

    VadInputData --> VadModel::infer
    VadModel::infer --> VadOutputData

    VadOutputData --> VadInterface2

    VadInterface2 --> VadOutputTopicData
    VadOutputTopicData --> VadNode_pub

    style VadNode_sub fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadInputTopicData fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadInterface fill:#633974,stroke:#512E5F,stroke-width:2px,color:#FFFFFF;

    style VadInputData fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
    style VadModel::infer fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;
    style VadOutputData fill:#1A5276,stroke:#154360,stroke-width:2px,color:#FFFFFF;

    style VadInterface2 fill:#633974,stroke:#512E5F,stroke-width:2px,color:#FFFFFF;
    style VadOutputTopicData fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
    style VadNode_pub fill:#943126,stroke:#78281F,stroke-width:2px,color:#FFFFFF;
```

- トピックの受け取り、座標変換などはインターフェース(`VadInterface`)で処理。

- 推論部分は完全に`VadModel`内に閉じた形で行われる。

---

### 想定Usecase

#### VADに新しいinputを追加する場合

- `VadModel`に新しい入力を追加．onnxを再学習

- `VadNode`が新しいtopicをsubscribeするように変更

- `VadInputTopicData`にtopicを追加

- `VadInterface`の入力変換処理を修正

- `VadInputData`にメンバを追加

## 「Autowareで使われる`camera_id`変更」という拡張に対して開いた設計

- 「Autowareで使われる`camera_id`変更」という拡張に対して開いた設計にしている。
- VADのcamera画像のid(順番)は，`VadInterface`のみに影響する。
  - `VadInputData`や`VadModel`には影響しない
- `autoware_to_vad_camera_mapping`のみを変更すれば、`camera_id`の変更ができる。

### 想定Usecase

#### VADの入力に使うカメラ画像のIDが変更された場合

- ROS param file([`ml_package_vad_tiny.param.yaml`](../config/ml_package_vad_tiny.param.yaml))の`autoware_to_vad_camera_mapping`を変更する
