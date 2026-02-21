#include <stdio.h>
#include <gst/gst.h>


// Callback function to dynamically link pads
void on_pad_added(GstElement* src, GstPad* new_pad, GstElement* sink) {
    GstPad* sink_pad = gst_element_get_static_pad(sink, "sink");
    
    // Check if the sink pad is already linked
    if (gst_pad_is_linked(sink_pad)) {
        g_object_unref(sink_pad);
        return;
    }
    
    // Attempt to link the newly created pad with the sink pad
    GstPadLinkReturn ret = gst_pad_link(new_pad, sink_pad);
    if (GST_PAD_LINK_FAILED(ret)) {
        g_printerr("Type is '%s' but link failed.\n",
            gst_structure_get_name(gst_caps_get_structure(gst_pad_get_current_caps(new_pad), 0)));
        }
        
        g_object_unref(sink_pad);
}
    int main(int argc, char* argv[]) {
        GMainLoop* loop;
        GstBus* bus;
        guint bus_watch_id;
    
        // Initialize GStreamer
        gst_init(&argc, &argv);
    
        // Create a main loop, which will run until we explicitly quit
        loop = g_main_loop_new(NULL, FALSE);
    
        // Create the elements
        GstElement* pipeline = gst_pipeline_new("video-player");
        GstElement* source = gst_element_factory_make("filesrc", "file-source1");
        GstElement* decoder = gst_element_factory_make("decodebin", "decode-bin1");
        GstElement* autovideosink = gst_element_factory_make("autovideosink", "video-output1");
    
        // Check that all elements were created successfully
        if (!pipeline || !source || !decoder || !autovideosink) {
            g_printerr("Not all elements could be created.\n");
            return -1;
        }
    
        // Set the source file location
        g_object_set(G_OBJECT(source), "location", "../resources/videos/big_buck_bunny_scene.mp4", NULL);
    
        // Add all elements to the pipeline
        gst_bin_add_many(GST_BIN(pipeline), source, decoder, autovideosink, NULL);
    
        // Link the source to the decoder (decoder will dynamically link to sink)
        if (!gst_element_link(source, decoder)) {
            g_printerr("Source and decoder could not be linked.\n");
            gst_object_unref(pipeline);
            return -1;
        }
    
        // Connect the pad-added signal for the decoder
        g_signal_connect(decoder, "pad-added", G_CALLBACK(on_pad_added), autovideosink);
    
        // Set the pipeline to the playing state
        gst_element_set_state(pipeline, GST_STATE_PLAYING);
    
        // Start the main loop and wait until it is quit (e.g., via EOS or an error)
        g_main_loop_run(loop);
    
        // Clean up: set the pipeline to NULL state, remove bus watch, and unref
        gst_element_set_state(pipeline, GST_STATE_NULL);
        gst_object_unref(GST_OBJECT(pipeline));
        g_main_loop_unref(loop);
    
        return 0;
    }