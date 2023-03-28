# Copyright 2023 Klepsydra Technologies AG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# -*- coding: utf-8 -*-

import sys
import unittest
import yaml

sys.path.insert(0, './kidl_data')
sys.path.insert(0, './preprocessor')
sys.path.insert(0, './processor')

from configuration import Configuration
from preprocessor import Preprocessor

from ros_mapper_processor import RosMapperProcessor

from jinja2 import Environment, FileSystemLoader, Template


TEMPLATE_PATH = './kpsr_codegen/templates'
CONF_PATH = "./kpsr_codegen/conf/"
INCLUDE_PATH = "./kpsr_codegen/tests/examples/"


class RosMapperProcessorTest(unittest.TestCase):

    def test_basic(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_mapper_processor = RosMapperProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "basic_class_with_ros_mdlw.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        ros_mapper_definition = ros_mapper_processor.process('BasicClass', class_definition_dict, '')

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('ros_mapper_template.h')

        print(template.render(definition=ros_mapper_definition))

    def test_related_classes(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_mapper_processor = RosMapperProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "class_with_ros_mdlw_and_related_classes.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        kidl_file = "basic_class_with_ros_mdlw.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        ros_mapper_definition = ros_mapper_processor.process('kpsr::codegen::ClassWithRosMdlwRelatedClasses',
                                                             class_definition_dict, '')

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('ros_mapper_template.h')

        print(template.render(definition=ros_mapper_definition))

    def test_parent_class(self):
        configuration = Configuration(CONF_PATH)
        preprocessor = Preprocessor(configuration)
        ros_mapper_processor = RosMapperProcessor(configuration)

        class_definition_dict = {}

        kidl_file = "class_with_ros_mdlw_and_parent_class.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        kidl_file = "basic_class_with_ros_mdlw.yaml"

        with open("%s%s" % (INCLUDE_PATH, kidl_file), 'r') as stream:
            try:
                class_definition_data = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                print(exc)

        class_definition = preprocessor.process(class_definition_data, False)
        class_definition_dict[class_definition.class_name] = class_definition

        ros_mapper_definition = ros_mapper_processor.process('kpsr::codegen::ClassWithParentClass',
                                                             class_definition_dict, '')

        env = Environment(
            loader=FileSystemLoader(TEMPLATE_PATH)
        )
        template = env.get_template('ros_mapper_template.h')

        print(template.render(definition=ros_mapper_definition))


if __name__ == '__main__':
    unittest.main()
