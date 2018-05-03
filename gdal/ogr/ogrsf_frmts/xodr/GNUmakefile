include ../../../GDALmake.opt
include XODRmake.opt

OBJ = xodr.o ogrxodrdriver.o ogrxodrdatasource.o ogrxodrlayer.o MatrixTransformations2D.o CubicPolynomials.o

CPPFLAGS	:=	$(GDAL_INCLUDE) $(XSD_INCLUDE) $(ODRSPIRAL_INCLUDE) $(XODR_INCLUDE) $(CPPFLAGS)

default:	$(O_OBJ:.o=.$(OBJ_EXT))

clean:
	rm -f *.o $(O_OBJ) $(PLUGIN_SO)

$(O_OBJ):	ogr_xodr.h

PLUGIN_SO	= ogr_XODR.$(SO_EXT)

plugin:		$(PLUGIN_SO)

$(PLUGIN_SO):	$(OBJ)
		$(LD_SHARED) $(LNK_FLAGS) $(OBJ) $(CONFIG_LIBS_INS) $(LIBS) $(ODRSPIRAL_LIB) $(XODR_LIB) \
		-o $(PLUGIN_SO)
	
plugin-install: plugin
		$(INSTALL_LIB) $(PLUGIN_SO) $(INST_LIB)/gdalplugins