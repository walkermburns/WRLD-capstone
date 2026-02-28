#ifndef VIDEOCOMPOSITE_H
#define VIDEOCOMPOSITE_H

#include <gst/gst.h>
#include <gst/gl/gl.h> // for GstGLShader

class VideoComposite {
public:
    VideoComposite();
    ~VideoComposite();

    // start the pipeline; on macOS this will be run from a secondary thread
    void start();

private:
    GstElement *pipeline;
    GstElement *stab[4];

    // shader uniforms
    float live_k1;
    float live_zoom;

    static gboolean update_fake_imu_cb(gpointer user_data);
    static gboolean on_draw_signal(GstElement *glfilter, GstGLShader *shader,
                                   guint texture, guint width, guint height,
                                   gpointer user_data);

    // entry-point used with gst_macos_main
    static void *run_pipeline(gpointer user_data);

    // When running on macOS, gst_macos_main sometimes doesn’t forward the
    // user_data pointer.  We keep a static fallback instance here so the
    // callback can still access the object.
    static VideoComposite *s_instance;
};

#endif // VIDEOCOMPOSITE_H