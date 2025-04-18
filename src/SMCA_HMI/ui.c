// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.6
// Project name: UI_SMCA

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
void ui_event_Screen1(lv_event_t * e);
lv_obj_t * ui_Screen1;
lv_obj_t * ui_ImageLogoUner;
lv_obj_t * ui_PanelTemperatura;
lv_obj_t * ui_LabelTemperatura;
lv_obj_t * ui_LabelTemperatura1;
lv_obj_t * ui_PanelHumedad;
lv_obj_t * ui_LabelHumedad;
lv_obj_t * ui_ValueHumi;
lv_obj_t * ui_PanelPesoMerma;
lv_obj_t * ui_LabelPeso;
lv_obj_t * ui_ValuePeso;
lv_obj_t * ui_LabelMerma;
lv_obj_t * ui_ValueMerma;
lv_obj_t * ui_PanelVelAire;
lv_obj_t * ui_LabelVelAire;
lv_obj_t * ui_ValueVelAire;
lv_obj_t * ui_PanelEtapa;
lv_obj_t * ui_LabelProducto;
lv_obj_t * ui_Dropdown2;
lv_obj_t * ui_Label2;
void ui_event_Button2(lv_event_t * e);
lv_obj_t * ui_Button2;
void ui_event_Button1(lv_event_t * e);
lv_obj_t * ui_Button1;
lv_obj_t * ui_LabelSTARTSTOP;


// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
lv_obj_t * ui_Screen2;
void ui_event_Button3(lv_event_t * e);
lv_obj_t * ui_Button3;
lv_obj_t * ui_LabelTituloScreen2;
lv_obj_t * ui_PanelSetTemperatura;
lv_obj_t * ui_Label3;
lv_obj_t * ui_SetTemperatura;
void ui_event_ButtonSetMasTemp(lv_event_t * e);
lv_obj_t * ui_ButtonSetMasTemp;
lv_obj_t * ui_LabelSetMasTemp;
void ui_event_ButtonSetMenosTemp(lv_event_t * e);
lv_obj_t * ui_ButtonSetMenosTemp;
lv_obj_t * ui_LabelSetMenosTemp;
lv_obj_t * ui_PanelSetHumedad;
lv_obj_t * ui_Label4;
lv_obj_t * ui_SetHumedad;
void ui_event_ButtonSetMenosHum(lv_event_t * e);
lv_obj_t * ui_ButtonSetMenosHum;
lv_obj_t * ui_LabelSetMenosHum;
void ui_event_ButtonSetMasHum(lv_event_t * e);
lv_obj_t * ui_ButtonSetMasHum;
lv_obj_t * ui_LabelSetMasHum;
lv_obj_t * ui_PanelSetMerma;
lv_obj_t * ui_Label5;
lv_obj_t * ui_SetMerma;
void ui_event_ButtonSetMenosMerma(lv_event_t * e);
lv_obj_t * ui_ButtonSetMenosMerma;
lv_obj_t * ui_LabelSetMenosMerma;
void ui_event_ButtonSetMasMerma(lv_event_t * e);
lv_obj_t * ui_ButtonSetMasMerma;
lv_obj_t * ui_LabelSetMasMerma;
lv_obj_t * ui_PanelSetVelAire;
lv_obj_t * ui_Label6;
lv_obj_t * ui_SetVelAire;
void ui_event_ButtonSetMenosVelAire(lv_event_t * e);
lv_obj_t * ui_ButtonSetMenosVelAire;
lv_obj_t * ui_LabelSetMenosVel;
void ui_event_ButtonSetMasVelAire(lv_event_t * e);
lv_obj_t * ui_ButtonSetMasVelAire;
lv_obj_t * ui_LabelSetMasVel;
lv_obj_t * ui____initial_actions0;
const lv_img_dsc_t * ui_imgset_setting_[1] = {&ui_img_setting_5720427_png};

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=1
    #error "LV_COLOR_16_SWAP should be 1 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_Screen1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen2_screen_init);
    }
}
void ui_event_Button2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen2, LV_SCR_LOAD_ANIM_MOVE_LEFT, 500, 0, &ui_Screen2_screen_init);
    }
}
void ui_event_Button1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        statusLED(e);
    }
}
void ui_event_Button3(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_Screen1_screen_init);
    }
}
void ui_event_ButtonSetMasTemp(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnMasTemp(e);
    }
}
void ui_event_ButtonSetMenosTemp(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnMenosTemp(e);
    }
}
void ui_event_ButtonSetMenosHum(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnMenosHum(e);
    }
}
void ui_event_ButtonSetMasHum(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnMasHum(e);
    }
}
void ui_event_ButtonSetMenosMerma(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnMenosMerma(e);
    }
}
void ui_event_ButtonSetMasMerma(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnMasMerma(e);
    }
}
void ui_event_ButtonSetMenosVelAire(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnMenosVel(e);
    }
}
void ui_event_ButtonSetMasVelAire(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);
    lv_obj_t * target = lv_event_get_target(e);
    if(event_code == LV_EVENT_CLICKED) {
        BtnMasVel(e);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_Screen1_screen_init();
    ui_Screen2_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_Screen1);
}
