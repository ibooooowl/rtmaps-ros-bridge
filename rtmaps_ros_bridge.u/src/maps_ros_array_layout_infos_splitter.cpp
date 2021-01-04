/*
 * Copyright (c) 2020 Intempora
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Intempora nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL INTEMPORA
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "maps_ros_array_layout_infos_splitter.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSros_array_layout_infos_splitter)
    MAPS_INPUT("array_layout",MAPSFilterROSArrayLayout,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSros_array_layout_infos_splitter)
	MAPS_OUTPUT("data_offset",MAPS::Integer32,NULL,NULL,1)
    MAPS_OUTPUT("dim_labels",MAPS::TextAscii,NULL,NULL,MAX_ARRAY_LAYOUT_LABEL_SIZE*MAX_ARRAY_LAYOUT_DIMENSIONS)
    MAPS_OUTPUT("dim_sizes",MAPS::Integer32,NULL,NULL,MAX_ARRAY_LAYOUT_DIMENSIONS)
    MAPS_OUTPUT("dim_strides",MAPS::Integer32,NULL,NULL,MAX_ARRAY_LAYOUT_DIMENSIONS)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSros_array_layout_infos_splitter)
    //MAPS_PROPERTY("pName",128,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSros_array_layout_infos_splitter)
    //MAPS_ACTION("aName",MAPSros_array_layout_infos_splitter::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ros_array_layout_infos_splitter) behaviour
MAPS_COMPONENT_DEFINITION(MAPSros_array_layout_infos_splitter,"ros_array_layout_infos_splitter","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions

void MAPSros_array_layout_infos_splitter::Birth()
{
}

void MAPSros_array_layout_infos_splitter::Core() 
{
	MAPSIOElt* ioeltin = StartReading(Input(0));
	if (ioeltin == NULL)
		return;

	ROSArrayLayout* layout = (ROSArrayLayout*)ioeltin->Data();

	MAPSIOElt* out_data_offset = StartWriting(Output(0));
	MAPSIOElt* out_labels = StartWriting(Output(1));
	MAPSIOElt* out_sizes = StartWriting(Output(2));
	MAPSIOElt* out_strides = StartWriting(Output(3));

	out_data_offset->Integer32() = layout->data_offset;

	int nb_dims = layout->nb_dims;
	MAPSStreamedString ss;
	for (int i=0; i<nb_dims; i++) {
		ss << layout->dim[i].label << ";";
		out_sizes->Integer32(i) = layout->dim[i].size;
		out_strides->Integer32(i) = layout->dim[i].stride;
	}
	char* labels_out = out_labels->TextAscii();
	MAPS::Strcpy(labels_out,ss);
	out_labels->VectorSize() = ss.Len();
	out_sizes->VectorSize() = nb_dims;
	out_strides->VectorSize() = nb_dims;

	out_data_offset->Timestamp() = ioeltin->Timestamp();
	out_labels->Timestamp() = ioeltin->Timestamp();
	out_sizes->Timestamp() = ioeltin->Timestamp();
	out_strides->Timestamp() = ioeltin->Timestamp();

	StopWriting(out_strides);
	StopWriting(out_sizes);
	StopWriting(out_labels);
	StopWriting(out_data_offset);
}

void MAPSros_array_layout_infos_splitter::Death()
{
}
