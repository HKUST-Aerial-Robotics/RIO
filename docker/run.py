#!/bin/env python3
import argparse
import shutil
import sys
import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def ArgsParser(argv):
    parser = argparse.ArgumentParser(description='Test RIO on a given dataset')
    parser.add_argument('-a', '--auto', action='store_true', help='Auto mode')
    parser.add_argument('-n', '--node', type=str, default='rio', help='Node name')
    parser.add_argument('-c', '--conf', type=str, default='config/ars548.yaml', help='Config file')
    parser.add_argument('-d', '--dataset', type=str, default='dataset/exp/Sequence_1.bag', help='Path to the dataset')
    parser.add_argument('-r', '--round', type=int, default=1, help='Number of rounds')
    parser.add_argument('-p', '--playSpeed', type=float, default=0.25, help='Play speed')

    return parser.parse_args(argv)

def RunEstimator(dataset, node, conf, round, playSpeed):
    dataset = os.path.join(BASE_DIR, dataset)
    CMD = 'bash -c "source /opt/ros/`rosversion -d`/setup.bash && source {}/../devel/setup.bash && roslaunch rio TestRadar.launch round:={} nodeName:={} configFile:={} playBagpath:={} playSpeed:={}"'.format(BASE_DIR, round, node, conf, dataset, playSpeed)
    os.system(CMD)

def runSingleRound(dataset, node, conf, round, playSpeed):
    RunEstimator(dataset, node, conf, round, playSpeed)

def main(argv):
    args = ArgsParser(argv)

    # Modify the ground truth topic
    if args.auto:
        if 'blocking' in args.dataset or 'square' in args.dataset:
            args.groundTruth = '/pos_vel_mocap/odom_TA'
        elif 'colo' in args.dataset:
            args.groundTruth = '/gt_pose'

    # Start testing
    for round in range(args.round):
        runSingleRound(args.dataset, args.node, args.conf, round, args.playSpeed)

if __name__ == '__main__':
    main(sys.argv[1:])
