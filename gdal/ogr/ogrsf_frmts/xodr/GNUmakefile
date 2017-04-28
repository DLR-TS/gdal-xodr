include ../../../GDALmake.opt
include XODRmake.opt

OBJ = odrSpiral.o xodr.o ogrxodrdriver.o ogrxodrdatasource.o ogrxodrlayer.o OpenDRIVE_1.4H.o

#CPPFLAGS	:=	-I.. -I../.. $(XSD_INC) $(CPPFLAGS)
CPPFLAGS	:=	$(GDAL_INCLUDE) $(XSD_INCLUDE) $(CPPFLAGS)

default:	$(O_OBJ:.o=.$(OBJ_EXT))

clean:
	rm -f *.o $(O_OBJ) $(PLUGIN_SO)

$(O_OBJ):	ogr_xodr.h

PLUGIN_SO	= ogr_XODR.$(SO_EXT)

plugin:		$(PLUGIN_SO)

$(PLUGIN_SO):	$(OBJ)
		$(LD_SHARED) $(LNK_FLAGS) $(OBJ) $(CONFIG_LIBS_INS) $(LIBS) \
		-o $(PLUGIN_SO)
	
plugin-install: plugin
		$(INSTALL_LIB) $(PLUGIN_SO) $(INST_LIB)/gdalplugins