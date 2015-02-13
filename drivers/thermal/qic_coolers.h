
struct tegra_cooling_device *qic_get_chg_cdev(void);
struct tegra_cooling_device *qic_get_fps_cdev(void);
struct dentry *get_g_qic_coolers_dir(void);
extern int get_g_fps_apply(void);
extern int get_g_fps_cooling_status(void);
extern int get_g_chg_apply(void);
extern int get_g_chg_cooling_status(void);