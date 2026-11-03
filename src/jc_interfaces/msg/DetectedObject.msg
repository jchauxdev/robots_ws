# Objeto detectado por un nodo de vision (jc_vision).
# header.frame_id indica el marco de 'pose' (p. ej. base_link o camera_optical_frame).
std_msgs/Header header
string label            # clase: "rojo", "aruco_7", "apple", ...
string shape            # forma geometrica: "cuadrado", "circulo", ... (vacio si no aplica)
float32 score           # confianza [0,1] (1.0 para detectores clasicos)
int32 u                 # centro en pixeles (columna)
int32 v                 # centro en pixeles (fila)
float32 width_px
float32 height_px
float32 angle_rad       # orientacion en el plano de la imagen (minAreaRect / marcador)
geometry_msgs/Pose pose # posicion 3D estimada (valida si pose_valid == true)
bool pose_valid
