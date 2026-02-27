import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge
import h5py
import numpy as np
import os
import time
import importlib
from typing import Dict, Any, List
from dataclasses import dataclass
from bisect import bisect_left
import threading
from datetime import datetime
from queue import Queue, Empty
from std_srvs.srv import Trigger
from data_collector.data_processor import DataProcessor
import yaml
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import time
import gc
import tracemalloc

def load_topics_config(config_path):
    with open(config_path) as f:
        config = yaml.safe_load(f)
    return config


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        self.config = load_topics_config(config_path)

        self.is_collecting = False
        self.timer = None
        self.dataset_cfg_dict = {}
        
        # 流式写入相关
        self.hf = None  # HDF5 文件句柄
        self.hdf5_datasets = {}  # 数据集引用
        self.frame_idx = 0  # 当前帧索引
        self.file_path = None  # 当前文件路径
        self.max_frames = 50000  # 预分配最大帧数（约16分钟 @50Hz）
        
        # 异步写入相关
        self.write_queue = None  # 写入队列
        self.write_thread = None  # 写入线程
        self.write_queue_size = 500  # 写入队列大小（可缓冲约10秒 @50Hz）
        self.dropped_frames = 0  # 丢帧计数
        self.write_stop_event = None  # 停止事件

        # 处理 data_dir：如果为 "auto" 或空，自动创建日期目录
        data_dir = self.config.get('data_dir', 'auto')
        if data_dir == 'auto' or not data_dir or (isinstance(data_dir, str) and data_dir.strip() == ''):
            # 从配置文件读取 base_dir，如果没有则使用默认值
            base_dir = self.config.get('base_dir', '/home/r1lite/GalaxeaDataset')
            # 若 base_dir 不存在（如移动硬盘未挂载），回退到本地目录
            fallback_base = '/home/r1lite/GalaxeaDataset'
            if not os.path.isdir(base_dir):
                self.get_logger().warn(
                    f"base_dir 不可用（路径不存在或非目录）: {base_dir}，使用 fallback: {fallback_base}"
                )
                base_dir = fallback_base
            date_str = datetime.now().strftime('%Y%m%d')
            data_dir = os.path.join(base_dir, date_str)
            self.config['data_dir'] = data_dir
            # 确保目录存在
            os.makedirs(data_dir, exist_ok=True)
            self.get_logger().info(f"📁 自动创建日期目录: {data_dir}")

        self.get_logger().info(f"data_dir: {self.config['data_dir']}")
        self.get_logger().info(f"collection_frequency: {self.config['collection_frequency']}")
        self.get_logger().info(f"max_queue_size: {self.config['max_queue_size']}")
        self.get_logger().info(f"datasets to collect: {self.config['datasets'].keys()}")

        self.bridge = CvBridge()
        self.processors: Dict[str, DataProcessor] = {}
        self.buffers: Dict[str, List] = {}
        self.locks: Dict[str, threading.Lock] = {}
        self.setup_processors()
        self.setup_subscriptions()

        self.srv_cb_group = MutuallyExclusiveCallbackGroup()
        self.start_srv = self.create_service(
            Trigger, 'start_collect', self.start_callback,
            callback_group=self.srv_cb_group)
        self.stop_srv = self.create_service(
            Trigger, 'stop_collect', self.stop_callback,
            callback_group=self.srv_cb_group)


    def setup_processors(self):
        datasets = self.config['datasets']
        for dataset_name, datset_value in datasets.items():
            try:
                module_name, class_name = datset_value['processor'].rsplit('.', 1)
                module = importlib.import_module(module_name)
                processor_class = getattr(module, class_name)
                self.processors[dataset_name] = processor_class(
                    datset_value.get('processor_config', {})
                )

                topic_name = datset_value['topic']
                self.buffers[topic_name] = []
                if topic_name not in self.locks.keys():
                    self.locks[topic_name] = threading.Lock()

                dataset_config = self.processors[dataset_name].get_dataset_config()
                self.get_logger().info(f"{dataset_name} config: {dataset_config}")
                self.dataset_cfg_dict[dataset_name] = dataset_config

            except Exception as e:
                self.get_logger().error(f"Failed to load processor for {dataset_name}: {str(e)}")
                raise

    def setup_subscriptions(self):
        topic_names = [ v['topic'] for _, v in self.config['datasets'].items() ]
        topic_configs = {
            topic_name: topic_cfg
            for topic_name, topic_cfg in self.config['topics'].items()
            if topic_name in topic_names
        }
        for topic_name, topic_cfg in topic_configs.items():
            msg_module, msg_type = topic_cfg['msg_type'].rsplit('.', 1)
            module = importlib.import_module(msg_module)
            msg_class = getattr(module, msg_type)

            self.create_subscription(
                msg_class,
                topic_name,
                self.create_callback(topic_name),
                5
            )

            self.get_logger().info(f"create sub {topic_name}")

    def create_callback(self, topic_name):
        def callback(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self.locks[topic_name]:
                self.buffers[topic_name].append((stamp, msg))
                max_q = self.config['max_queue_size']
                if len(self.buffers[topic_name]) > max_q:
                    self.buffers[topic_name].pop(0)

        return callback

    def start_callback(self, request, response):
        tracemalloc.start()
        # request is empty for Trigger
        if self.is_collecting:
            response.success = False
            response.message = "Already collecting data"
            return response

        # Get frequency from parameters instead of request
        frequency = self.config['collection_frequency']
        interval = 1.0 / frequency
        
        # 创建 HDF5 文件（流式写入）
        try:
            self._create_hdf5_file()
        except Exception as e:
            response.success = False
            response.message = f"Failed to create HDF5 file: {str(e)}"
            return response
        
        # 初始化异步写入
        self.write_queue = Queue(maxsize=self.write_queue_size)
        self.write_stop_event = threading.Event()
        self.dropped_frames = 0
        self.write_thread = threading.Thread(target=self._write_worker, daemon=True)
        self.write_thread.start()
        
        self.get_logger().info(f"Start streaming to {self.file_path} (async mode, queue_size={self.write_queue_size})")
        self.is_collecting = True
        self.frame_idx = 0
        self.timer = self.create_timer(interval, self.collect_data)

        response.success = True
        response.message = f"Started collecting at {frequency}Hz (async streaming mode)"
        return response
    
    def _create_hdf5_file(self):
        """创建 HDF5 文件和可扩展数据集"""
        data_dir = self.config['data_dir']
        os.makedirs(data_dir, exist_ok=True)
        timestamp = int(time.time())
        self.file_path = os.path.join(data_dir, f'collection_{timestamp}.hdf5')
        
        self.hf = h5py.File(self.file_path, 'w')
        self.hdf5_datasets = {}
        
        for dataset_name, dataset_cfg in self.dataset_cfg_dict.items():
            shape = dataset_cfg['shape']
            dtype = dataset_cfg['dtype']
            
            # 创建可扩展数据集
            dataset = self.hf.create_dataset(
                dataset_name,
                shape=(self.max_frames, *shape),
                dtype=dtype,
                chunks=(1, *shape),
                maxshape=(None, *shape)  # 允许无限扩展
            )
            dataset.attrs['description'] = dataset_cfg['description']
            self.hdf5_datasets[dataset_name] = dataset
            self.get_logger().info(f"Created dataset {dataset_name}: shape=({self.max_frames}, {shape})")
        
        # 保存采集频率
        self.hf.create_dataset(
            'collection_frequency',
            data=self.config['collection_frequency'],
            dtype='f'
        )
    
    def _write_worker(self):
        """异步写入线程：从队列中取数据写入 HDF5"""
        write_count = 0
        while not self.write_stop_event.is_set():
            try:
                # 带超时的取数据，以便检查停止事件
                frame_idx, data_dict = self.write_queue.get(timeout=0.1)
                
                # 检查是否需要扩展数据集
                if frame_idx >= self.max_frames:
                    self._expand_datasets()
                
                # 写入所有数据集
                for dataset_name, data in data_dict.items():
                    self.hdf5_datasets[dataset_name][frame_idx] = data
                
                write_count += 1
                if write_count % 50 == 0:
                    queue_size = self.write_queue.qsize()
                    self.get_logger().info(f"Written {write_count} frames, queue: {queue_size}/{self.write_queue_size}")
                
                self.write_queue.task_done()
                
            except Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Write error: {e}")
        
        # 处理队列中剩余的数据
        remaining = 0
        while not self.write_queue.empty():
            try:
                frame_idx, data_dict = self.write_queue.get_nowait()
                if frame_idx >= self.max_frames:
                    self._expand_datasets()
                for dataset_name, data in data_dict.items():
                    self.hdf5_datasets[dataset_name][frame_idx] = data
                remaining += 1
                self.write_queue.task_done()
            except Empty:
                break
        
        if remaining > 0:
            self.get_logger().info(f"Flushed {remaining} remaining frames")

    def stop_callback(self, request, response):
        if not self.is_collecting:
            response.success = False
            response.message = "Not currently collecting"
            return response

        self.is_collecting = False
        if self.timer:
            self.timer.cancel()

        try:
            # 停止写入线程并等待队列清空
            self.get_logger().info(f"Stopping... Queue has {self.write_queue.qsize()} pending frames")
            self.write_stop_event.set()
            
            if self.write_thread and self.write_thread.is_alive():
                self.write_thread.join(timeout=30)  # 最多等待30秒
                if self.write_thread.is_alive():
                    self.get_logger().warn("Write thread did not finish in time!")
            
            # 裁剪数据集到实际大小并关闭文件
            self._finalize_hdf5_file()
            
            snapshot = tracemalloc.take_snapshot()
            top_stats = snapshot.statistics('lineno')
            self.get_logger().info(f"Memory top 5: {top_stats[:5]}")
            
            gc.collect()
            
            # 报告丢帧情况
            drop_msg = ""
            if self.dropped_frames > 0:
                drop_msg = f" (WARNING: {self.dropped_frames} frames dropped!)"
            
            response.success = True
            response.message = f"Saved {self.frame_idx} frames to {self.file_path}{drop_msg}"
        except Exception as e:
            response.success = False
            response.message = str(e)
            # 确保资源被释放
            if self.write_stop_event:
                self.write_stop_event.set()
            if self.hf is not None:
                try:
                    self.hf.close()
                except:
                    pass
                self.hf = None

        return response
    
    def _finalize_hdf5_file(self):
        """裁剪数据集到实际大小并关闭文件"""
        if self.hf is None:
            return
        
        self.get_logger().info(f"Finalizing HDF5 file with {self.frame_idx} frames...")
        
        # 裁剪每个数据集到实际帧数
        for dataset_name, dataset in self.hdf5_datasets.items():
            original_shape = dataset.shape
            new_shape = (self.frame_idx, *original_shape[1:])
            dataset.resize(new_shape)
            self.get_logger().info(f"{dataset_name}: {original_shape} -> {new_shape}")
        
        # 关闭文件
        self.hf.close()
        self.hf = None
        self.hdf5_datasets = {}
        
        self.get_logger().info(f"Saved to {self.file_path}")

    def collect_data(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        max_age = self.config['max_age']
        
        # 收集本帧所有数据
        frame_data = {}

        for dataset_name, dataset_cfg in self.config['datasets'].items():
            topic_name = dataset_cfg['topic']
            with self.locks[topic_name]:
                buffer = self.buffers[topic_name]
                if not buffer:
                    self.raise_error(f"No data in {topic_name}")
                    return
                
                stamps = [s for s, _ in buffer]
                idx = bisect_left(stamps, current_time)
                closest = self.find_closest(stamps, idx, current_time)

                if closest is None:
                    self.raise_error(f"No closet data in {topic_name}")

                delta_time = abs(buffer[closest][0] - current_time)
                if self.frame_idx % 50 == 0:  # 每秒打印一次（50Hz）
                    queue_size = self.write_queue.qsize() if self.write_queue else 0
                    self.get_logger().info(f"Frame {self.frame_idx}: queue={queue_size}/{self.write_queue_size}, dropped={self.dropped_frames}")
                    
                if delta_time > max_age:
                    self.raise_error(f"Stale data in {topic_name}, delta time: {delta_time}")
                    return

                _, msg = buffer[closest]
                processed_data = self.processors[dataset_name].process(msg)
                frame_data[dataset_name] = processed_data

                message = self.processors[dataset_name].get_message()
                if message != '':
                    self.get_logger().warn('*'*50)
                    self.get_logger().warn(message)
                    self.get_logger().warn('*'*50)

                # always save one frame at least in buffer
                self.buffers[topic_name] = buffer[closest:]

        # 将数据放入写入队列（非阻塞）
        try:
            self.write_queue.put_nowait((self.frame_idx, frame_data))
        except:
            # 队列满了，丢弃本帧
            self.dropped_frames += 1
            if self.dropped_frames % 10 == 1:
                self.get_logger().warn(f"Queue full! Dropped {self.dropped_frames} frames so far")
        
        self.frame_idx += 1

        time_cost = self.get_clock().now().nanoseconds / 1e9 - current_time
        time_interval = 1.0 / self.config['collection_frequency']

        if time_cost > time_interval:
            self.get_logger().warn(f"Frame {self.frame_idx}: process_time={time_cost:.4f}s SLOW!")
    
    def _expand_datasets(self):
        """扩展数据集容量"""
        new_max = self.max_frames + 10000
        self.get_logger().info(f"Expanding datasets from {self.max_frames} to {new_max} frames...")
        
        for dataset_name, dataset in self.hdf5_datasets.items():
            old_shape = dataset.shape
            new_shape = (new_max, *old_shape[1:])
            dataset.resize(new_shape)
        
        self.max_frames = new_max

    def find_closest(self, stamps, idx, target):
        if idx == 0:
            return 0 if stamps else None
        elif idx == len(stamps):
            return idx - 1
        else:
            before = stamps[idx - 1]
            after = stamps[idx]
            return idx - 1 if (target - before) <= (after - target) else idx

    def raise_error(self, msg):
        self.get_logger().error(msg)
        self.is_collecting = False
        if self.timer:
            self.destroy_timer(self.timer)
        raise RuntimeError(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
