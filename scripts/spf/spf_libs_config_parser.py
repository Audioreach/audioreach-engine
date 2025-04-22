#===============================================================================
#
# SPF shared library config parser
#
# GENERAL DESCRIPTION parses the json files containing spf library configuration and outputs
#                     (.) dictionary needed for building shared libs,
#                     (.) adds env variable for static libs,
#                     (.) generates information needed for amdb_registration/loading,
#                         CPP files for static and dynamic modules.
#                     (.) generates XML for ARCT,
#
#
#
# Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear
#===============================================================================
import re
import shutil
import os
import errno
import glob


def spf_libs_config_parser(env, is_unit_test=False):
   from collections import OrderedDict;
   import os;
   tgt_specific_folder = os.environ.get('TGT_SPECIFIC_FOLDER')
   project_source_dir = os.environ.get('PROJECT_SOURCE_DIR')
   c_file_dir = project_source_dir + '/fwk/spf/amdb/autogen/' + tgt_specific_folder + "/"
   project_bin_dir = os.environ.get('PROJECT_BINARY_DIR')
   json_file_dir= "%s%s" %(project_bin_dir,"/libs_cfg/*.json")
   spf_cfg_autogen_folder = 'spf_cfg_autogen'
   qact_xml_file_name=spf_cfg_autogen_folder+'/spf_module_build_info.xml';
   static_source_file_name = 'spf_static_build_config'
   shared_source_file_name = 'spf_shared_build_config'
   shared_source_file_name_priv = 'spf_shared_build_config_private'
   static_source_file_name_priv = 'spf_static_build_config_private'

   inc_header_files=['amdb_api.h', 'capi.h', 'amdb_autogen_def.h']
   #### constants
   MODULE_TYPE_MAP=OrderedDict([('generic',2), ('decoder',3), ('encoder',4),('converter',5),('packetizer',6),('depacketizer',7),('detector',8),('generator',9),('pp',10),('end_point',11),('framework',1)])
   INTERFACE_TYPE_MAP=OrderedDict([('virtual_stub',1),('capi',2)]);#('capi',1),('appi',0)
   STATIC_BUILD_TYPES=['STATIC_BUILD_STRIP', 'STATIC_BUILD_NO_STRIP',
                     'STATIC_BUILD_STRIP_ONLY', 'STATIC_BUILD_STUB', 'STATIC_BUILD_VIRTUAL_STUB'];
   SHARED_BUILD_TYPES=['SHARED_BUILD_STRIP', 'SHARED_BUILD_NO_STRIP',
                     'SHARED_NO_BUILD']; #SHARED_NO_BUILD: not built but maybe added to AMDB if AMDB_info is present.
   STUB_BUILD_TYPES=['STATIC_BUILD_STUB', 'STATIC_BUILD_VIRTUAL_STUB'];

   STATIC_BUILD_TYPES_ENV_SUFFIX_MAP = {'STATIC_BUILD_STRIP':'_STRIP',
                     'STATIC_BUILD_NO_STRIP':None,  #for NO_STRIP nothing to do
                     'STATIC_BUILD_STRIP_ONLY':'_STRIP_ONLY',
                     'STATIC_BUILD_STUB':'_STUB',
                     'STATIC_BUILD_VIRTUAL_STUB':'_STUB',}

   ## when GEN_SHARED_LIBS is not in the env, then below takes effect.
   SHARED_BUILD_TYPE_TO_STATIC_MAP={'STATIC_BUILD_STRIP':'STATIC_BUILD_STRIP',
                        'STATIC_BUILD_NO_STRIP':'STATIC_BUILD_NO_STRIP',
                        'STATIC_BUILD_STRIP_ONLY':'STATIC_BUILD_STRIP_ONLY',
                        'STATIC_BUILD_STUB':'STATIC_BUILD_STUB',
                        'SHARED_BUILD_STRIP':'STATIC_BUILD_STRIP',
                        'SHARED_BUILD_NO_STRIP':'STATIC_BUILD_NO_STRIP',
                        'SHARED_NO_BUILD':'SHARED_NO_BUILD',
                        'STATIC_BUILD_VIRTUAL_STUB':'STATIC_BUILD_VIRTUAL_STUB'}

   class ModuleIdClass:
      def __init__(self,  module_type, mid):
         self.mid = int(mid.strip(),0)
         self.module_type = module_type.strip().lower();
      def __str__(self):
         return 'Module ID: (mtype=%s, mid=%s). '% (self.module_type, hex(self.mid))
      def __key(self):
         return (self.mid) #whether ,m-id is uniq or not doesnt depend on capi or capiv2
      def __eq__(self, other):
         return (self.__key()==other.__key());
      def __hash__(self):
         h=hash(self.__key());
         return h;

   class BuildInfoClass:
      def __init__(self, libname, build, is_amdb_info_present,lib_major_ver=0,
                     lib_minor_ver=0, depends_on_list=[],
                     security_info=None, pkg_info=None,island_info=None, is_private=None):
         if (len(libname)==0) or (len(build)==0):
            s='BuildInfoClass: lib name or build cannot be empty';
            raise LibCfgParserError(s);
         if build not in STATIC_BUILD_TYPES+SHARED_BUILD_TYPES:
            s='SpfLibsCfg Error: BuildInfoClass: lib name %s has unsupported build %s'%(libname, build);
            raise LibCfgParserError(s);
         self.libname = libname;
         self.build = build;
         self.lib_major_ver = lib_major_ver;
         self.lib_minor_ver = lib_minor_ver;
         self.depends_on_list = depends_on_list;
         self.is_amdb_info_present = is_amdb_info_present;
         self.pkg_info = pkg_info;
         self.security_info = security_info;
         self.island_info = island_info;
         self.is_private = is_private
      def __str(self):
         return 'BuildInfoClass: lib %s, build %s, ver %d.%d'%(self.libname,self.build,self.lib_major_ver,self.lib_minor_ver)
      def __key(self):
         return (self.libname) #every 2 buildInfo elements must have these items different
      def __eq__(self, other):
         return (self.__key()==other.__key());
      def __hash__(self):
         h=hash(self.__key());
         return h;
      def eq_in_all_respects(self, other):
        return ((self.libname == other.libname) and (self.build == other.build) and \
            (self.lib_major_ver == other.lib_major_ver) and (self.lib_minor_ver == other.lib_minor_ver) and \
            (self.depends_on_list == other.depends_on_list) and \
            (self.pkg_info == other.pkg_info) and (self.security_info == other.security_info) and (self.island_info == other.island_info) )

      def get_so_file_name(self):
         if self.build in STATIC_BUILD_TYPES:
            return None;
         # so file name is required even for no-build.
         return self.libname+'.so.'+str(self.lib_major_ver);
      def _populate_so_build_system(self, shared_lib_dict):
         if self.build in STATIC_BUILD_TYPES:
            return;
         if not self.build=='SHARED_NO_BUILD':
            strip_str='_SO';
            if self.build=='SHARED_BUILD_STRIP':
               strip_str='_STRIP_SO'
            uses_key='USES_'+self.libname.upper()+strip_str;

            depends_on_list=[];
            if len(self.depends_on_list)>0:
               for i in self.depends_on_list:
                  depends_on_list.append(str(i).strip());

            if uses_key in shared_lib_dict:
               s='SpfLibsCfg: Warning: .so file '+self.libname+' was already added for build. Fix if this is a mistake.'
               print(s)
            else:
               ## only for non-mobile pass security info etc
               is_qurt_used = False; #this flag is no longer used
               if 'USES_NON_MOBILE' in env:
                  shared_lib_dict[uses_key]=[ [self.lib_major_ver, self.lib_minor_ver], [is_qurt_used],depends_on_list,self.security_info,self.pkg_info];
                  print('spf', '%s [ [%d, %d], [%d], %s, %s, %s, %s]'%(uses_key,self.lib_major_ver,self.lib_minor_ver,is_qurt_used,str(depends_on_list),self.security_info,self.pkg_info, self.island_info) )
               else:
                  shared_lib_dict[uses_key]=[ [self.lib_major_ver, self.lib_minor_ver], [is_qurt_used],depends_on_list];
                  print('spf', '%s [ [%d, %d], [%d], %s, %s]'%(uses_key,self.lib_major_ver,self.lib_minor_ver,is_qurt_used,str(depends_on_list),self.island_info) )

      def _populate_static_build_system(self, env):
         if self.build not in STATIC_BUILD_TYPES:
            return;
         #import pdb;pdb.set_trace()
         k=STATIC_BUILD_TYPES_ENV_SUFFIX_MAP[self.build];
         if not k==None:
            s='USES_'+self.libname.upper()+k;
            s_I='USES_'+self.libname.upper()+"_ISLAND"+k;
            if not is_unit_test:
               #import pdb;pdb.set_trace()
               s1='env.Replace('+s+'=True)'
               exec(s1) in locals()
               if "USES_AUDIO_IN_ISLAND" not in env or self.island_info == False:
                  s2='env.Replace('+s_I+'=True)'
                  exec(s2) in locals()

            print('spf', 'SpfLibsCfg: env.Replace(%s=True)'%(s) )
            if "USES_AUDIO_IN_ISLAND" not in env or self.island_info == False:
                 print('spf', 'SpfLibsCfg: env.Replace(%s=True)'%(s_I))
         print('spf', 'SpfLibsCfg: lib %s: %s '%(self.libname.upper(), self.build) )
         if "USES_AUDIO_IN_ISLAND" not in env or self.island_info == False:
             print('spf', 'SpfLibsCfg: lib %s: %s '%(self.libname.upper()+"_ISLAND", self.build) )

      def _populate_island_build_system(self, env):
         #import pdb;pdb.set_trace()
         if self.build not in STATIC_BUILD_TYPES or "USES_AUDIO_IN_ISLAND" not in env or self.island_info == False or self.island_info == "False":
            return;

            #For non-island also apply build types needed...
         k=STATIC_BUILD_TYPES_ENV_SUFFIX_MAP[self.build];
         #if "USES_AUDIO_IN_ISLAND" in env:
         #if self.island_info == True:

         if k != None:
               s='USES_'+self.libname.upper()+"_ISLAND"+k;
         else:
               s='USES_'+self.libname.upper()+"_ISLAND";
         s_p='USES_'+self.libname.upper()+"_ISLAND"+"_PLINK_ISLAND";
         #import pdb;pdb.set_trace()
         s2='env.Replace('+s+'=True)'
         s2_p='env.Replace('+s_p+'=True)'
         exec(s2) in locals()
         exec(s2_p) in locals()
         print('spf', 'SpfLibsCfg Island: env.Replace(%s=True)'%(s) )
         print('spf', 'SpfLibsCfg Island: env.Replace(%s=True)'%(s_p) )
         print('spf', 'SpfLibsCfg Island: lib %s: %s '%(self.libname.upper(), self.build) )

      def populate_build_system(self, shared_lib_dict, env):
         self._populate_so_build_system(shared_lib_dict)
         self._populate_static_build_system(env)
         self._populate_island_build_system(env)

      # TODO:this doesn't count revisions of modules.
      def count_build_type(self, build_type_counter_no_amdb_info, build_type_counter_with_amdb_info):
         if self.is_amdb_info_present:
            if self.build in build_type_counter_with_amdb_info:
               build_type_counter_with_amdb_info[self.build]+=1;
            else:
               build_type_counter_with_amdb_info[self.build]=1;
         else:
            if self.build in build_type_counter_no_amdb_info:
               build_type_counter_no_amdb_info[self.build]+=1;
            else:
               build_type_counter_no_amdb_info[self.build]=1;
      def __str__(self):
         return "BuildInfoClass: lib name %s, build %s, lib_major_ver %d, lib_minor_ver %d, depends_on_list %s" %(self.libname, self.build, self.lib_major_ver, self.lib_minor_ver, str(self.depends_on_list))

   class ModuleInfoClass:
      def __init__(self, interface_type, module_name, build, filename='', tag='', fmtid1=None, fmtid2=None, is_test_module=False, qact_module_type=None):
         self.interface_type = interface_type.strip().lower()
         self.module_name=module_name;
         self.build=build;
         self.filename=filename; #so_file valid for shared lib only.
         self.tag=tag.strip();
         self.fmtid1=fmtid1;
         self.fmtid2=fmtid2;
         self.is_test_module = is_test_module;
         self.qact_module_type = qact_module_type;
      def get_built_type(self):
         if self.build in 'STATIC_BUILD_VIRTUAL_STUB' or self.interface_type in 'virtual_stub':
            print('SpfLibsCfg: virtual_stub '+str(self))
            return 'VIRTUAL-STUB'
         elif self.build in STATIC_BUILD_TYPES:
               return 'STATIC';
         else:
            return 'DYNAMIC'; #note that NO_BUILD also becomes DYNAMIC. static modules need not be kept with rev_num. Hence NO_BUILD is never for static.
      def need_to_appear_in_qact(self, env):
         #virtual stub modules, framework modules, test modules
         if self.build in 'STATIC_BUILD_VIRTUAL_STUB' or self.interface_type in 'virtual_stub' or \
            self.is_test_module:
            return False;
         return True;
      def is_new_rev_needed(self, other):
         return not (self.filename == other.filename);
      def get_fmt_id1_str(self):
         if self.fmtid1 == None:
            return 'NOT_APPLICABLE'
         else:
            return self.fmtid1
      def get_fmt_id2_str(self):
         if self.fmtid2 == None:
            return 'NOT_APPLICABLE'
         else:
            return self.fmtid2
      def __str__(self):
         return 'AMDB Info: (Itype %s, Module Name %s, Filename=%s, tag=%s, build %s). '% (self.   interface_type,\
            self.module_name,str(self.filename),\
            self.tag,self.build)  ;

   class LibCfgParserError(Exception):
      def __init__(self, value):
         self.value = value
      def __str__(self):
         return repr(self.value)

   def map_island_info(lib):
       #import pdb;pdb.set_trace()
       island_info = False
       found = False
       if 'island' in lib:
          island_config = lib['island']
          island_info = island_config
          if island_config.__class__ == dict:

              island_info = island_config
              for k in island_config.keys():
                  k=k.strip();
                  if not k == 'DEFAULT':
                     if env.get(k.upper()) == True or env.get(k.lower()) == True:
                        island_info = island_config[k].strip();
                        found=True;
                        break;
              if not found:
                 island_info = island_config['DEFAULT'].strip();
                 #print "This  is ",island_info
       return island_info


   def map_build_input_to_built_str(lib_name, build_in, is_amdb_info_present):
      if build_in.__class__ == dict:
         if 'DEFAULT' not in build_in:
            s='the build dict does not contain default value for lib '+lib_name
            raise LibCfgParserError(s);
         found=False;
         for k in build_in.keys():
            k=k.strip();
            if not k == 'DEFAULT':
               if env.get(k.upper()) == True or env.get(k.lower()) == True:
                  build_in = build_in[k];
                  found=True;
                  break;
         if not found:
            build_in = build_in['DEFAULT'].strip();
            #print build_in

      build_in=build_in.strip();
      ## those libs which are configured as shared libs must be built as static if 'GEN_SHARED_LIBS' is not defined
      if 'GEN_SHARED_LIBS' not in env:
         # some libs like supporting .so files need not be built when building without GEN_SHARED_LIBS in env.
         # if they are built then symbol clash may occur.
         # such libs dont have AMDB info. that's how they are identified.
         if build_in in SHARED_BUILD_TYPES:
            if not is_amdb_info_present:
               print('SpfLibsCfg: '+lib_name+' with build='+build_in+' has no AMDB info. Will not be built without GEN_SHARED_LIBS.')
               build_in = 'SHARED_NO_BUILD'
         build_in=SHARED_BUILD_TYPE_TO_STATIC_MAP[build_in]
      return build_in;

   #===============================================================================
   # Parse the json files
   # mod_info: {m-id:{rev_num:(module-info)}}
   # build_info_set : set(build_info)
   #===============================================================================
   def parse_each_json(json_info, build_info_set, mod_info):
      for lib in json_info:
         #import pdb;pdb.set_trace()
         try:
            lib_name = lib['lib_name'].strip();
            is_amdb_info_present='amdb_info' in lib and len(lib['amdb_info'])>0;

            b='STATIC_BUILD_NO_STRIP';
            if 'build' in lib:
               b=lib['build'];
            build = map_build_input_to_built_str(lib_name, b, is_amdb_info_present);

            if is_amdb_info_present and build=='STATIC_BUILD_STUB':
               s='SpfLibsCfg Error: Library %s has amdb_info but is specified for static build stub. This is not supported. Use STATIC_BUILD_VIRTUAL_STUB.'%(lib_name)
               raise LibCfgParserError(s);
            island_info = map_island_info(lib)
            maj_version,min_version=0,0;
            # for shared libs, version is mandatory.
            if build in SHARED_BUILD_TYPES: #directly read so that if user doesnt provide KeyError will occur
               maj_version = lib['lib_major_ver'];
               min_version = lib['lib_minor_ver'];
            else:
               if 'lib_major_ver' in lib:
                  maj_version = lib['lib_major_ver'];
               if 'lib_minor_ver' in lib:
                  min_version = lib['lib_minor_ver'];

            depends_on=[];
            if 'depends_on' in lib:
               depends_on = lib['depends_on'];

            pkg_info=None
            if 'pkg_info' in lib:
               #import pdb;pdb.set_trace()
               pkg_info = lib['pkg_info'].strip();

            security_info = None;
            if 'security_info' in lib:
               #import pdb;pdb.set_trace()
               security_info = lib['security_info'];
               security_info = {k.strip():v.strip() for k,v in security_info.iteritems()}
               ## extra checks to ensure manual error is not made.
               if not (build in SHARED_BUILD_TYPES or build in STUB_BUILD_TYPES):
                  s='SpfLibsCfg Error: Library %s has security_info but not built as shared or as stub (security can be achieved only for shared libs). build=%s'%(lib_name, build)
                  raise LibCfgParserError(s);
               if None == pkg_info:
                  s='SpfLibsCfg Error: Library %s has security_info but not pkg_info'%(lib_name)
                  raise LibCfgParserError(s);

            if None == security_info and not None == pkg_info:
               s='SpfLibsCfg Error: Library %s has pkg_info but not security_info'%(lib_name)
               raise LibCfgParserError(s);
            is_private = False
            if "is_private" in lib:
                is_private = True
            else:
                is_private = False
            #every item in json must have build-info
            build_info = BuildInfoClass(lib_name, build, is_amdb_info_present, maj_version,
               min_version, depends_on, security_info, pkg_info, island_info, is_private);

            if build_info in build_info_set:
               already_added=build_info_set[build_info]; #build_info.hash is used as key
               if not build_info.eq_in_all_respects(already_added):
                  s='SpfLibsCfg Error: Library %s is already added for build but all fields are not matching'% \
                     (lib_name);
                  raise LibCfgParserError(s)
            else:
               build_info_set[build_info] = build_info #using orderedDict to implement orderedSet
            is_test_module = False;
            if 'is_test_module' in lib:
               is_test_module = lib['is_test_module'];

            #amdb_info is optional. all shared lib or all static libs need not have.
            if is_amdb_info_present:
               rev_num=1;
               if 'rev_num' in lib['amdb_info']:
                  rev_num = lib['amdb_info']['rev_num'];
               itype = lib['amdb_info']['itype'].strip().lower();

               mtype = lib['amdb_info']['mtype'].strip().lower();
               mid = lib['amdb_info']['mid'].strip().lower();
               tag='' #tag can be empty for 'virtual stubs'
               if 'tag' in lib['amdb_info']:
                  tag = lib['amdb_info']['tag'].strip();
               module_name = lib['amdb_info']['module_name'].strip();
               fmtid1 = None; fmtid2 = None;
               if 'fmt_id1' in lib['amdb_info']:
                  fmtid1 = lib['amdb_info']['fmt_id1'].strip();
               if 'fmt_id2' in lib['amdb_info']:
                  fmtid2 = lib['amdb_info']['fmt_id2'].strip();
               qact_module_type = mtype;
               if 'qact_module_type' in lib['amdb_info'] and lib['amdb_info']['qact_module_type'].strip() != "":
                  qact_module_type = lib['amdb_info']['qact_module_type'].strip();

               if mtype in ['decoder', 'encoder' , 'converter', 'depacketizer'] and None==fmtid1:
                  s='SpfLibsCfg Error: module of type %s must specify fmtid1'%(mtype)
                  raise LibCfgParserError(s);
               if mtype in ['converter'] and None==fmtid2:
                  s='SpfLibsCfg Error: module of type %s must specify fmtid2'%(mtype)
                  raise LibCfgParserError(s);

               if build=='STATIC_BUILD_VIRTUAL_STUB':
                  print('SpfLibsCfg Info: Library %s is virtual-stub, hence forcing itype of module %s from %s to virtual_stub'%(lib_name,module_name,itype))
                  itype='virtual_stub' #if itype is virtual_stub, but build is not, then it's ok to build as we are not going to use what's built.

               if (itype not in INTERFACE_TYPE_MAP.keys()):
                   s='SpfLibsCfg Error: unknown interface type '+itype
                   raise LibCfgParserError(s);

               if (mtype not in MODULE_TYPE_MAP.keys()):
                   s='SpfLibsCfg Error: unknown module_type '+mtype
                   raise LibCfgParserError(s);

               if (qact_module_type not in MODULE_TYPE_MAP.keys()):
                   s='SpfLibsCfg Error: unknown qact_module_type '+qact_module_type
                   raise LibCfgParserError(s);
               if (qact_module_type in ['framework']):
                   s='SpfLibsCfg Error: qact_module_type cannot be %s module'%(qact_module_type)
                   raise LibCfgParserError(s);

               if not (build == 'STATIC_BUILD_VIRTUAL_STUB'):
                  if tag==None or len(tag)==0:
                     s='SpfLibsCfg Error: when build type is %s and amdb_info is present, tag must be present for module-name %s'+(build, module_name)
                     raise LibCfgParserError(s);

               so_filename = build_info.get_so_file_name();


               new_m_id = ModuleIdClass(mtype, mid);
               new_mod_info = ModuleInfoClass( itype, module_name, build, so_filename, tag, fmtid1, fmtid2, is_test_module, qact_module_type);

               if new_m_id in mod_info:
                  if rev_num in mod_info[new_m_id]:
                     s='SpfLibsCfg Error: %s Revision %d already exists.'%(str(new_m_id), rev_num);
                     raise LibCfgParserError(s);
                  else:
                     #if this & prev or this & next versions are already in mod_info, and they are not same file name then, the revision is fine. otherwise, revision incr was mistakenly done .
                     if ( ( (rev_num-1) in mod_info[new_m_id] and new_mod_info.is_new_rev_needed( mod_info[new_m_id][rev_num-1] )) or \
                          ( (rev_num+1) in mod_info[new_m_id] and new_mod_info.is_new_rev_needed( mod_info[new_m_id][rev_num+1] )) ):
                           mod_info[new_m_id][rev_num] = new_mod_info;
                     else:
                        s='SpfLibsCfg Error: This and previous revision are same for %s, rev %d'%(str(new_m_id), rev_num);
                        raise LibCfgParserError(s);
               else:
                  mod_info[new_m_id] = {rev_num:new_mod_info}

         except KeyError as e:
             print('SpfLibsCfg: KeyError '+ str(e))
             print('SpfLibsCfg: Mandatory keys are not provided')
             import traceback
             traceback.print_exc()
             raise e;
      return ;

   #===============================================================================
   # Print mod_info
   #===============================================================================
   def print_mod_info(mod_info):
      for m in mod_info:
         for r in mod_info[m]:
            print('SpfLibsCfg: %s, rev=%d, %s' % (str(m), r, str(mod_info[m][r])))

   def write_c_macros(file):
      for h in inc_header_files:
         file.write('#include "%s"\n' %(h));
      file.write('\n');
      for i in INTERFACE_TYPE_MAP:
         file.write('#define %-10s %d\n' %(i.upper(), INTERFACE_TYPE_MAP[i]));
      file.write('\n');
      for m in MODULE_TYPE_MAP:
         file.write('#define %-10s %d\n' %(m.upper(), MODULE_TYPE_MAP[m]));
      file.write('\n');

   #mod_info {m-id:{rev_num:(module-info)}}
   def write_autogen_files(static_source_file, static_header_file, shared_source_file, shared_header_file, mod_info, shared_source_file_priv,
							shared_header_file_priv, static_source_file_priv, static_header_file_priv):
      ## first loop to get some info.
      prereq_mid_list=OrderedDict(); # {module-id-class:[module-id-class]}
      for m in mod_info:
         # sort with highest revision first.
         revisions=sorted(mod_info[m], reverse=True)
         is_latest_rev=True;
         last_r=0
         for r in revisions:
            if not is_latest_rev:
               is_latest_rev=False;
               if last_r != r+1:
                  s='SpfLibsCfg: Error: module %s has skipped versions between %d and %d'%(m,last_r,r)
                  raise LibCfgParserError(s);
            last_r=r

      s='/** Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved. */\n' + \
        '/** SPDX-License-Identifier: BSD-3-Clause-Clear */\n' + \
        '/** ***** This is an autogenerated file ***** **/\n' + \
        '/** ***** DO NOT EDIT THIS FILE ***** **/\n\n'
      static_source_file.write(s);
      shared_source_file.write(s);
      static_header_file.write(s);
      shared_header_file.write(s);
      if env.IsBuildInternal:
        shared_header_file_priv.write(s);
        shared_source_file_priv.write(s);
        shared_header_file_priv.write('#ifndef __SPF_SHARED_BUILD_CONFIG_H_\n#define __SPF_SHARED_BUILD_CONFIG_H_\n\n');
        static_header_file_priv.write(s);
        static_source_file_priv.write(s);
        static_header_file_priv.write('#ifndef __SPF_STATIC_PRIV_BUILD_CONFIG_H_\n#define __SPF_STATIC_PRIV_BUILD_CONFIG_H_\n\n');

      static_header_file.write('#ifndef __SPF_STATIC_BUILD_CONFIG_H_\n#define __SPF_STATIC_BUILD_CONFIG_H_\n\n');
      shared_header_file.write('#ifndef __SPF_SHARED_BUILD_CONFIG_H_\n#define __SPF_SHARED_BUILD_CONFIG_H_\n\n');

      write_c_macros(static_header_file);
      static_header_file.write('#ifdef __cplusplus \nextern "C" {\n#endif //__cplusplus \n');
      static_source_file.write('#include "spf_begin_pragma.h" \n');
      static_source_file.write('#include "'+static_source_file_name+'.h"\n\n');

      write_c_macros(shared_header_file);
      shared_header_file.write('#ifdef __cplusplus \nextern "C" {\n#endif //__cplusplus \n');
      shared_source_file.write('#include "spf_begin_pragma.h" \n');
      shared_source_file.write('#include "'+shared_source_file_name+'.h"\n\n');

      if env.IsBuildInternal:
        write_c_macros(shared_header_file_priv);
        shared_header_file_priv.write('#ifdef __cplusplus \nextern "C" {\n#endif //__cplusplus \n');
        write_c_macros(static_header_file_priv);
        static_header_file_priv.write('#ifdef __cplusplus \nextern "C" {\n#endif //__cplusplus \n');
      if env.IsBuildInternal:
        shared_source_file_priv.write('#include "spf_begin_pragma.h" \n');
        shared_source_file_priv.write('#include "'+shared_source_file_name_priv+'.h"\n\n');
        static_source_file_priv.write('#include "spf_begin_pragma.h" \n');
        static_source_file_priv.write('#include "'+static_source_file_name_priv+'.h"\n\n');

      temp_file_name_dict={};
      temp_tag_dict={};

      mod_counter = {}; #static, dynamic capi, capi
      for itype in INTERFACE_TYPE_MAP.keys():
         mod_counter['STATIC ' + itype] = 0;
         mod_counter['DYNAMIC ' + itype] = 0;
         mod_counter['DYNAMIC_PRIVATE ' + itype] = 0;
         mod_counter['STATIC_PRIVATE ' + itype] = 0;
         close_sh_parethesis=True;
         if itype=='capi':
            shared_source_file.write('const amdb_dynamic_capi_module_t amdb_spf_dynamic_capi_modules[] = \n{\n');
            if env.IsBuildInternal:
                shared_source_file_priv.write('const amdb_dynamic_capi_module_t amdb_private_dynamic_capi_modules[] = \n{\n');
                static_source_file_priv.write('const amdb_static_capi_module_t amdb_private_static_capi_modules[] = \n{\n');
            static_source_file.write('const amdb_static_capi_module_t amdb_spf_static_capi_modules[] = \n{\n');
         elif itype=='virtual_stub':
            static_source_file.write('const amdb_module_id_t amdb_virtual_stub_modules[] = \n{\n');
            close_sh_parethesis=False
         else:
            s='SpfLibsCfg Error: module %s has unhandled itype %s'%(m,itype)
            raise LibCfgParserError(s);

         for m in mod_info:
            # sort with highest revision first.
            revisions=sorted(mod_info[m], reverse=True)
            for r in revisions:
               if not mod_info[m][r].interface_type == itype:
                  continue;
               if mod_info[m][r].get_built_type()=='DYNAMIC':
                  mod_counter['DYNAMIC ' + itype] += 1;
                  fn_c_var=mod_info[m][r].filename.upper().replace('.','_').replace('-','_');
                  tag_c_var=mod_info[m][r].tag.upper();
                  if fn_c_var not in temp_file_name_dict:
                     temp_file_name_dict[fn_c_var]=0;
                     shared_header_file.write('const char %-40s   = {%-40s};\n'%(fn_c_var+'[]','"'+mod_info[m][r].filename+'"'))
                  if tag_c_var not in temp_tag_dict:
                     temp_tag_dict[tag_c_var]=0;
                     shared_header_file.write('const char %-40s   = {%-40s};\n'%(tag_c_var+'[]','"'+mod_info[m][r].tag+'"'))
                  if itype=='capi':
                     shared_source_file.write('{%10s, %10s, %2d, %-40s, %-40s},\n' %(m.module_type.upper(),
                        hex(m.mid), r, fn_c_var, tag_c_var));
                  elif itype=='virtual_stub':
                     #this cannot happen as virtual-stubs are classified not as dynamic
                     pass
                  else:
                     s='SpfLibsCfg Error: module %s has unhandled itype %s'%(m,itype)
                     raise LibCfgParserError(s);

               else: #static/virtual_stubs
                  if (len(revisions)>1):
                     s='SpfLibsCfg Error: module %s has more than one revision for static'%(m)
                     raise LibCfgParserError(s);
                  mod_counter['STATIC ' + itype] += 1; #includes virtual stubs
                  first_func='';
                  second_func='';
                  decl_first_func='';
                  decl_second_func='';
                  if itype=='capi':
                      if m.module_type == 'framework':
                        first_func = 'NULL';
                        second_func = 'NULL';
                      else:
                        first_func=mod_info[m][r].tag+'_get_static_properties'
                        second_func=mod_info[m][r].tag+'_init'
                        decl_first_func='capi_err_t %s (capi_proplist_t *init_set_proplist, capi_proplist_t *static_proplist);'%(first_func)
                        decl_second_func='capi_err_t %s (capi_t* _pif, capi_proplist_t *init_set_proplist);'%(second_func)
                      static_source_file.write('{%10s, %10s, %-40s, %-40s},\n' %(m.module_type.upper(), hex(m.mid), first_func,second_func ));
                  elif itype=='virtual_stub':
                     static_source_file.write('{%10s, %10s},\n' %(m.module_type.upper(), hex(m.mid)));
                  else:
                     s='SpfLibsCfg Error: module %s has unhandled itype %s'%(m,itype)
                     raise LibCfgParserError(s);

                  #declaration of the functions
                  if (len(decl_first_func)>0):
                     static_header_file.write(decl_first_func+'\n')
                     static_header_file.write(decl_second_func+'\n\n')

         static_source_file.write('\n};\n\n');
         if close_sh_parethesis:
            shared_source_file.write('\n};\n\n');
            if env.IsBuildInternal:
                shared_source_file_priv.write('\n};\n\n');
                static_source_file_priv.write('\n};\n\n');
         if itype=='capi':
            static_source_file.write('const uint32_t amdb_spf_num_static_capi_modules = %d;\n\n'%(mod_counter['STATIC ' + itype]));
            shared_source_file.write('const uint32_t amdb_spf_num_dynamic_capi_modules = %d;\n\n'%(mod_counter['DYNAMIC ' + itype]));
            if env.IsBuildInternal:
                shared_source_file_priv.write('const uint32_t amdb_num_dynamic_private_capi_modules = %d;\n\n'%(mod_counter['DYNAMIC_PRIVATE ' + itype]));
                static_source_file_priv.write('const uint32_t amdb_num_static_private_capi_modules = %d;\n\n'%(mod_counter['STATIC_PRIVATE ' + itype]));
         elif itype=='virtual_stub':
            static_source_file.write('const uint32_t amdb_num_virtual_stub_modules = %d;\n\n'%(mod_counter['STATIC ' + itype]));

      """
      Incase of ARE of APPS when spf fwk is compiled standalone without ADSP build,
      define and initialized below variables to default values to get there definition
      as they are defined extern in amdb source file.
      """
      if env.SPF_FWK_COMPILATION:
        shared_source_file.write('const amdb_dynamic_capi_module_t amdb_dynamic_capi_modules[] = \n{\n\n};\n\n');
        static_source_file.write('const amdb_static_capi_module_t amdb_static_capi_modules[] = \n{\n\n};\n\n');
        static_source_file.write('const uint32_t amdb_num_static_capi_modules = %d;\n\n'%(0));
        shared_source_file.write('const uint32_t amdb_num_dynamic_capi_modules = %d;\n\n'%(0));

      shared_source_file.write('#include "spf_end_pragma.h"\n');
      if env.IsBuildInternal:
        shared_source_file_priv.write('#include "spf_end_pragma.h"\n');
        shared_header_file_priv.write('#ifdef __cplusplus \n} /*extern c*/ \n#endif //__cplusplus \n\n#endif /*__SPF_SHARED_BUILD_CONFIG_H_*/\n\n'); #extern c
        static_source_file_priv.write('#include "spf_end_pragma.h"\n');
        static_header_file_priv.write('#ifdef __cplusplus \n} /*extern c*/ \n#endif //__cplusplus \n\n#endif /*__SPF_STATIC_PRIVATE_BUILD_CONFIG_H_*/\n\n'); #extern c

      static_source_file.write('#include "spf_end_pragma.h"\n');
      shared_header_file.write('#ifdef __cplusplus \n} /*extern c*/ \n#endif //__cplusplus \n\n#endif /*__SPF_SHARED_BUILD_CONFIG_H_*/\n\n'); #extern c
      static_header_file.write('#ifdef __cplusplus \n} /*extern c*/ \n#endif //__cplusplus \n\n#endif /*__SPF_STATIC_BUILD_CONFIG_H_*/\n\n'); #extern c

   class ComponentInfo:

     def __init__(self):

        print("Getting comp specific info")
        component_name = env["AU_NAME"]

        build_root = os.environ.get('${BUILD_ROOT}')
        proc_type = build_root.split('/')[-1]
        if proc_type in ["modem_proc","adsp_proc","slpi_proc","cdsp_proc"]:
           print("Setting up proc_mode as", proc_type)
        else:
            raise Exception('Error: Invalid Proc Name..!')

        if not component_name:
           raise Exception('Error: Invalid component Name..!')

        self.component_name =    component_name
        self.proc_type   =    proc_type
        #self.auto_dirs = self.get_auto_dirs() #get auto dirs

        print("Component name", component_name)

   #===============================================================================
   # Write ARCT XML
   # # mod_info=dictionary of modules. indexed by m-id. Each m-id contains a dict of mod-info. {m-id:{rev_num:(module-info)}}
   #===============================================================================
   def write_qact_xml(qact_xml_file, mod_info):
      build_id_tag1='UNIT TEST BUILD'
      build_id_tag2='UNIT TEST BUILD'
      if not is_unit_test:
         build_id_tag1=str(os.environ.get('${ENGG_TIME_STAMP}'))
         build_id_tag2=str(os.environ.get('${QCOM_TIME_STAMP}'))

      qact_xml_file.write('<?xml version="1.0"?>\n');
      qact_xml_file.write('<!--\n\n   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved. \n'+
      '   SPDX-License-Identifier: BSD-3-Clause-Clear \n\n'+
      '   DESCRIPTION: Build information of the modules in SPF. \n'+
      '                This file lists the built-in modules in SPF build\n'+
      '                For each module it lists whether the module is built as static module or as dynamic module(.so file) \n'+
      '                For encoders, decoders etc it also informs the supported format-id.\n'+
      '\n'+
      '   ***** This is an autogenerated file ***** \n'+
      '   ***** DO NOT EDIT THIS FILE ***** \n\n-->\n\n')

      #proc_name = 'APM_PROC_DOMAIN_ID_SDSP'
      #proc_id = 4;
      proc_name = 'APM_PROC_DOMAIN_ID_ADSP'
      proc_id = 1;
      qact_xml_file.write('<SPF-DB maj_ver="1"  min_ver=  "1">\n');
      qact_xml_file.write('<HEADER version="1">\n');
      qact_xml_file.write('   <BUILDID> %s </BUILDID>\n' %(build_id_tag2));
      qact_xml_file.write('   <BUILDID> %s </BUILDID>\n' %(build_id_tag1));
      qact_xml_file.write('   <PROC_DOMAIN_NAME> %s </PROC_DOMAIN_NAME>\n' %(proc_name));
      qact_xml_file.write('   <PROC_DOMAIN_ID> %s </PROC_DOMAIN_ID>\n' %(proc_id));
#     qact_xml_file.write('   <PROC_DOMAIN_NAME> %s </PROC_DOMAIN_NAME>\n' %(a["proc_name"]));
#     qact_xml_file.write('   <PROC_DOMAIN_ID> %s </PROC_DOMAIN_ID>\n' %(a["proc_id"]));
      qact_xml_file.write('</HEADER>\n');

      for m in mod_info:
         revisions=sorted(mod_info[m], reverse=True)
         r=revisions[0]; #select top revision. only preload flag is altered by revision.
         if mod_info[m][r].need_to_appear_in_qact(env):
            if mod_info[m][r].module_name==None or len(mod_info[m][r].module_name)==0:
                s='SpfLibsCfg Error: Module needs to have a valid name. lib_name %s' % str(mod_info[m][r])
                raise LibCfgParserError(s);
            qact_xml_file.write('<MODINFO version="1">\n');
            #no need of interface type as it can change.
            qact_xml_file.write('   <MTYPE> '+mod_info[m][r].qact_module_type+' </MTYPE>\n');
            qact_xml_file.write('   <MID> '+hex(m.mid)+' </MID>\n');
            qact_xml_file.write('   <NAME> '+mod_info[m][r].module_name+' </NAME>\n');
            qact_xml_file.write('   <FMTID1> '+mod_info[m][r].get_fmt_id1_str()+' </FMTID1>\n');
            qact_xml_file.write('   <FMTID2> '+mod_info[m][r].get_fmt_id2_str()+' </FMTID2>\n');
            qact_xml_file.write('   <BUILDTYPE>  '+mod_info[m][r].get_built_type()+' </BUILDTYPE>\n');
            qact_xml_file.write('</MODINFO>\n');

      qact_xml_file.write('</SPF-DB>\n');

   def check_at_least_one_rev_built(mod_info):
      for m in mod_info:
         at_least_one_rev_b = False;
         for r in mod_info[m]:
            if not mod_info[m][r].build == 'SHARED_NO_BUILD':
               at_least_one_rev_b=True;
               break;
         if not at_least_one_rev_b:
            s='Not even one revision built for module %s'%(str(m));
            raise LibCfgParserError(s)

   #===============================================================================
   # checks if the root dir is under hap
   #===============================================================================
   def is_build_under_hap(env):
      if is_unit_test:
         return False;
      return re.search('hap', env.RealPath(c_file_dir))

   #===============================================================================
   # parse_cfg_and_gen_output
   #===============================================================================
   def parse_cfg_and_gen_output(env):
      chipset=os.environ.get('CHIPSET')
      if None == chipset:
         print('SpfLibsCfg: Error: chipset input not given')
         return None;

      only_populate_env = False;
      if not is_unit_test and env.GetOption('clean') and not env.GetOption('cleanpack'):
         #clean all outputs, dont return beacause pack utility needs the same lib cfg as was used for build.
         if os.path.exists(spf_cfg_autogen_folder):
            shutil.rmtree(spf_cfg_autogen_folder)
         if os.path.exists(c_file_dir):
            shutil.rmtree(c_file_dir)
         only_populate_env = True;

      shared_lib_dict=OrderedDict();

      if not is_build_under_hap(env):
         if not only_populate_env:
            if not os.path.exists(spf_cfg_autogen_folder):
                try:
                    os.makedirs(spf_cfg_autogen_folder)
                except OSError as exc:
                    if exc.errno != 17:
                        print(exc.errno)
                        raise exc
                    pass

            if not os.path.exists(c_file_dir):
                try:
                    os.makedirs(c_file_dir)
                except OSError as exc:
                    if exc.errno != 17:
                        raise exc
                    pass

            try:
               qact_xml=open(qact_xml_file_name, 'w');
               static_source_file=open(os.path.join(c_file_dir,static_source_file_name)+'.c', 'w');
               static_header_file=open(os.path.join(c_file_dir,static_source_file_name)+'.h', 'w');
               shared_source_file=open(os.path.join(c_file_dir,shared_source_file_name)+'.c', 'w');
               shared_header_file=open(os.path.join(c_file_dir,shared_source_file_name)+'.h', 'w');
               if env.IsBuildInternal:
                  shared_source_file_priv=open(os.path.join(c_file_dir,shared_source_file_name_priv)+'.c', 'w');
                  shared_header_file_priv=open(os.path.join(c_file_dir,shared_source_file_name_priv)+'.h', 'w');
                  static_source_file_priv=open(os.path.join(c_file_dir,static_source_file_name_priv)+'.c', 'w');
                  static_header_file_priv=open(os.path.join(c_file_dir,static_source_file_name_priv)+'.h', 'w');
               else:
                  shared_source_file_priv, shared_header_file_priv = (0,0)
                  static_source_file_priv, static_header_file_priv = (0,0)
            except IOError as e:
               print('SpfLibsCfg: Error opening files : '+str(e))
               raise e

      json_files=[f for f in glob.glob(json_file_dir)]

      for f in json_files:
        print(f)


      import json
      mod_info=OrderedDict(); # dictionary of modules. indexed by m-id. Each m-id contains a dict of mod-info. {m-id:{rev_num:(module-info)}}
      build_info_set=OrderedDict(); #Using OrderedDict for implementing orderedSet. set of buildInfo
      build_type_counter_no_amdb_info={};
      build_type_counter_with_amdb_info={};
      for json_file in json_files:
         #print 'SpfLibsCfg: Working on file '+json_file
         try:
            fp=open(json_file);
         except IOError as e:
            print('SpfLibsCfg: Error opening JSON file : '+json_file+'. Ignoring ')
            continue

         try:
            sh=json.load(fp);
         except ValueError as e:
            print('SpfLibsCfg: Error loading JSON file : '+json_file)
            raise e

         fp.close();
         if (len(sh)>0):
            print('SpfLibsCfg: Parsing json file '+json_file)
            parse_each_json(sh, build_info_set, mod_info);
         else:
            print('SpfLibsCfg: json file '+json_file+' is empty')

      # write what's necessary for build-system
      for buildInfo in build_info_set:
         #buildInfo.count_build_type(build_type_counter_no_amdb_info,build_type_counter_with_amdb_info);
         buildInfo.populate_build_system(shared_lib_dict, env);

      #voice modules fail this check since they add custom modules without building them.
      #check_at_least_one_rev_built(mod_info)

      if not is_build_under_hap(env):
         if not only_populate_env:
            write_qact_xml(qact_xml, mod_info);
            qact_xml.close()
            # write C files for all modules.
            write_autogen_files(static_source_file, static_header_file, shared_source_file, shared_header_file, mod_info, shared_source_file_priv, shared_header_file_priv, static_source_file_priv, static_header_file_priv);

            static_source_file.close();
            static_header_file.close();
            shared_source_file.close();
            shared_header_file.close();
            if env.IsBuildInternal:
                shared_source_file_priv.close();
                shared_header_file_priv.close();
                static_source_file_priv.close();
                static_header_file_priv.close();

      if 'GEN_SHARED_LIBS' in env and not is_unit_test :
         env.Replace(SHARED_LIB_DICT=shared_lib_dict)


      return shared_lib_dict

   #===============================================================================
   # Core functionality for the scons script
   #===============================================================================
   #print env flags
   if 'USES_SPF_SHARED_LIBS' in env:
      print('SpfLibsCfg: USES_SPF_SHARED_LIBS in env')
   if 'GEN_SHARED_LIBS' in env:
      print('SpfLibsCfg: GEN_SHARED_LIBS in env')
   if not is_unit_test and env.GetOption('clean'):
      print('SpfLibsCfg: clean command')

   shared_lib_dict = parse_cfg_and_gen_output(env);

   return shared_lib_dict

#===============================================================================
# prints shared lib dict
#===============================================================================
def print_shared_lib_dict(shared_lib_dict):
   print('SpfLibsCfg: shared_lib_dict : ')
   for item in shared_lib_dict.keys():
      print(item, shared_lib_dict[item])

#===============================================================================
# Main function for unit testing
#===============================================================================

if __name__ == '__main__':
   from collections import OrderedDict;
   env = OrderedDict();

   """
   IsBuildInternal flag decides to generate amdb autogen entry for private modules.
   Currently the build does not support flag value as False and leads to missing
   symbols error during linking. Set value true for the flag to enable stub
   functionality.
   """
   env.IsBuildInternal = True

   """
   When spf framework is compiled seperately along with ADSP image then AMDB requires
   two list to store the entries for SPF module and module part of ADSP image.
   Incase of ARE of APPS where spf fwk is compiled standalone without ADSP image,
   module list for ADSP moduele is not defined, so define SPF_FWK_COMPILATION flag
   to TRUE to initialize the variable with default values to get there definition
   as they are defined extern in amdb source file.
   """
   env.SPF_FWK_COMPILATION = True

   """
   Define "GEN_SHARED_LIBS" environment variable to TRUE to add module entry to AMDB
   dynamic capi module list. If not defined, dynamic modules entry is added to AMDB
   static capi module list.
   """
   env["GEN_SHARED_LIBS"] = True;

   shared_lib_dict = spf_libs_config_parser(env, True);