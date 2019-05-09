float distance_parcourue_cm(float x_init, float y_init) {
// cette routine calcule la distance euclidienne entre la position en cours,
// ,donnée par xt et yt  - coordonnées de la position actuelle du robot
// issues odométries (var globales) et les coordonnées de la position fournies 

return sqrt((xt-x_init)*(xt-x_init)+(yt-y_init)*(yt-y_init));

}

